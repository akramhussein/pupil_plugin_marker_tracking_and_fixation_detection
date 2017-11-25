#
#   Pupil Plugin - Marker Tracking and Fixation Detection
#
#   Copyright 2017 Akram Hussein
#

import sys
import os
import platform
import cv2
import numpy as np
import serial

from led_control import LedControl
from fixation_detector import gaze_dispersion, fixation_from_data
from square_marker_detect_ import (
    detect_markers,
    detect_markers_robust,
    draw_markers,
    m_marker_to_screen
)

from collections import deque
from file_methods import Persistent_Dict, load_object
from pyglui.cygl.utils import draw_points, draw_polyline, draw_polyline_norm, draw_circle, RGBA
from pyglui import ui
from pyglui.pyfontstash import fontstash
from pyglui.ui import get_opensans_font_path
from glfw import *
from OpenGL.GL import *
from plugin import Plugin
from methods import denormalize
from copy import deepcopy

import logging
logger = logging.getLogger(__name__)

class Serial():
    ''' handle serial port communication '''
    def __init__(self, port, baudrate):
        self.ser = serial.Serial()
        self.ser.baudrate = baudrate
        self.ser.port = port
        logging.info('Opened serial port ' + str(self.ser))
        self.ser.open()

    def write(self, data):
        if self.ser.is_open:
            self.ser.write(data)

    def close(self):
        self.ser.close()


class Marker_Tracker_And_Fixation_Detector(Plugin):
    icon_chr = chr(0xec07)
    icon_font = 'pupil_icons'

    def __init__(
            self,
            g_pool,
            min_marker_perimeter=100,
            invert_image=False,
            robust_detection=True,
            show_marker_direction=False,
            show_marker_names=True,
            hit_region_extension_percentage=0.0,
            calibration_marker_id=0,
            indicator_max_distance_to_marker=1000,
            angle_adjustment=0,
            max_fiducual_id=6,
            serial_port='COM1',
            max_dispersion=3.0,
            min_duration=300,
            confidence_threshold=0.75):

        super().__init__(g_pool)

        # Marker Tracking
        self.visible_markers = []

        self.min_marker_perimeter = min_marker_perimeter
        self.invert_image = invert_image
        self.robust_detection = robust_detection
        self.show_marker_direction = show_marker_direction
        self.show_marker_names = show_marker_names
        self.hit_region_extension_percentage = hit_region_extension_percentage
        self.hit_region_percentage = 0.0
        self.calibration_marker_id = 0

        self.aperture = 11
        self.min_id_confidence = 0.0
        self.img_shape = None

        # LEDs
        self.max_fiducual_id = max_fiducual_id
        self.indicator_max_distance_to_marker = 1000
        self.angle_adjustment = angle_adjustment
        self.serial_port = serial_port
        self.led_control = LedControl()

        # attempt to setup Serial port
        try:
            # self.serial = arduinoserial.SerialPort(self.serial_port, 115200)
            self.serial = Serial(self.serial_port, 115200)
        except serial.serialutil.SerialException:
            self.serial = None

        # Fixation Detection
        self.queue = deque()
        self.min_duration = min_duration
        self.max_dispersion = max_dispersion
        self.confidence_threshold = confidence_threshold
        self.recent_fixation = None

        # UI
        self.running = True
        self.menu = None
        self.button = None

        self.glfont = fontstash.Context()
        self.glfont.add_font('opensans', get_opensans_font_path())
        self.glfont.set_size(22)
        self.glfont.set_color_float((1.0, 1.0, 1.0, 1.0))

    def init_ui(self):
        def set_max_marker_id(new_max_marker_id):
            try:
                self.max_fiducual_id = int(new_max_marker_id)
            except ValueError:
                logger.error("Max fiducial id must be an integer. {} is not".format(new_marker_count))

        def set_calibration_marker_id(new_calibration_marker_id):
            try:
                self.calibration_marker_id = int(new_calibration_marker_id)
            except ValueError:
                logger.error("Calibration marker id must be an integer. {} is not".format(new_calibration_marker_id))

        def set_port(new_port):
            try:
                self.serial_port = new_port
                self.serial = Serial(self.serial_port, 115200)
            except ValueError:
                logger.error("Serial port must be an integer: {}. Unable to setup LEDs.".format(new_port))
            except serial.serialutil.SerialException:
                logger.error("Serial port not valid: {}. Unable to setup LEDs.".format(new_port))
                self.serial = None

        self.add_menu()

        self.button = ui.Thumb('running', self, label='M', hotkey='m')
        self.button.on_color[:] = (.1, 1., .2, .8)
        self.g_pool.quickbar.append(self.button)

        self.menu.elements[:] = []

        self.menu.label = 'Marker Tracker and Fixation Detector'

        self.menu.append(ui.Info_Text(
            'This plugin detects and tracks fiducial markers, highlighting when they are fixated on.'))

        # Marker Trackings
        tracking_menu = ui.Growing_Menu("Marker Tracking")
        tracking_menu.collapsed = False

        tracking_menu.append(ui.Switch('robust_detection',
                                       self, label='Robust detection'))
        tracking_menu.append(ui.Switch('invert_image', self,
                                       label='Use inverted markers'))
        tracking_menu.append(ui.Slider('min_marker_perimeter',
                                       self, label="Min Marker Perimeter", step=1, min=10, max=100))
        tracking_menu.append(ui.Switch('show_marker_direction', self,
                                       label='Show Marker Direction'))
        tracking_menu.append(ui.Switch('show_marker_names', self,
                                       label='Show Marker Names'))
        tracking_menu.append(ui.Slider('hit_region_extension_percentage',
                                       self, label="Hit Region Extension %", step=1, min=0, max=400))

        tracking_menu.append(ui.Text_Input('calibration_marker_id', self, setter=set_calibration_marker_id, label='Calibration Marker ID'))

        self.menu.append(tracking_menu)

        # Fixation Detection
        fixation_detection_menu = ui.Growing_Menu("Fixation Detection")
        fixation_detection_menu.collapsed = False

        fixation_detection_menu.append(ui.Slider('max_dispersion', self, min=0.01, step=0.1, max=5.,
                                                 label='Maximum Dispersion [degrees]'))
        fixation_detection_menu.append(ui.Slider('min_duration', self, min=10, step=10, max=1500,
                                                 label='Minimum Duration [milliseconds]'))

        fixation_detection_menu.append(ui.Slider('confidence_threshold', self,
                                                 min=0.0, max=1.0, label='Confidence Threshold'))

        self.menu.append(fixation_detection_menu)

        # LED Control
        led_control_menu = ui.Growing_Menu("LED Control")
        led_control_menu.collapsed = False

        led_control_menu.append(ui.Text_Input('max_fiducual_id', self, setter=set_max_marker_id, label='Max Marker ID'))
        led_control_menu.append(ui.Text_Input('serial_port', self, setter=set_port, label='Serial Port'))
        led_control_menu.append(ui.Slider('indicator_max_distance_to_marker',
                                       self, label="Indicator Max Distance to Marker", step=10, min=1, max=10000))
        led_control_menu.append(ui.Slider('angle_adjustment',
                                       self, label="Angle Adjustment", step=1, min=0, max=360))

        self.menu.append(led_control_menu)

    def deinit_ui(self):
        self.g_pool.quickbar.remove(self.button)
        self.button = None
        self.glfont = None
        self.remove_menu()

    def get_init_dict(self):
        return {
            'min_marker_perimeter': self.min_marker_perimeter,
            'invert_image': self.invert_image,
            'robust_detection': self.robust_detection,
            'show_marker_direction': self.show_marker_direction,
            'show_marker_names': self.show_marker_names,
            'hit_region_extension_percentage': self.hit_region_extension_percentage,
            'calibration_marker_id': self.calibration_marker_id,
            'indicator_max_distance_to_marker': self.indicator_max_distance_to_marker,
            'angle_adjustment': self.angle_adjustment,
            'max_fiducual_id': self.max_fiducual_id,
            'serial_port': self.serial_port,
            'max_dispersion': self.max_dispersion,
            'min_duration': self.min_duration,
            'confidence_threshold': self.confidence_threshold
        }

    def recent_events(self, events):
        # Get current frame
        frame = events.get('frame')
        if not frame:
            return

        self.img_shape = frame.height, frame.width, 3

        if self.running:
            self.button.status_text = 'tracking running'
        else:
            self.button.status_text = 'tracking paused'
            self.clear_leds()

        if self.running:
            # Invert frame if necessary
            gray = frame.gray
            if self.invert_image:
                gray = 255-gray

            # Get visible markers and visualize them
            # From: https://github.com/pupil-labs/pupil/pull/872
            if self.robust_detection:
                self.visible_markers_flipped = detect_markers_robust(
                    gray,
                    grid_size=5,
                    aperture=self.aperture,
                    prev_markers=self.visible_markers,
                    true_detect_every_frame=3,
                    min_marker_perimeter=self.min_marker_perimeter)
            else:
                self.visible_markers_flipped = detect_markers(
                    gray,
                    grid_size=5,
                    aperture=self.aperture,
                    min_marker_perimeter=self.min_marker_perimeter)

            # The pixel representation has a flipped y-axis. We repair that for our
            # internal use.
            self.visible_markers = deepcopy(self.visible_markers_flipped)

            for m in self.visible_markers_flipped:
                m['fixated'] = False
                m['within_roi'] = False
                m['distance'] = 0.0
                m['angle'] = 0.0

            # Check for calibration marker
            if self.calibration_marker_id in list(map(lambda x: x['id'], self.visible_markers_flipped)):
                self.running = False
                self.notify_all({'subject': 'calibration.should_start'})
                return

            if len(self.visible_markers_flipped) > 0:
                events['fixations'] = []
                gaze = events['gaze_positions']

                self.queue.extend((gp for gp in gaze if gp['confidence'] > self.confidence_threshold))

                try:  # use newest gaze point to determine age threshold
                    age_threshold = self.queue[-1]['timestamp'] - self.min_duration / 1000.
                    while self.queue[1]['timestamp'] < age_threshold:
                        self.queue.popleft()  # remove outdated gaze points
                except IndexError:
                    pass

                gaze_3d = [gp for gp in self.queue if '3d' in gp['base_data'][0]['method']]
                use_pupil = len(gaze_3d) > 0.8 * len(self.queue)

                base_data = gaze_3d if use_pupil else self.queue

                if len(base_data) <= 2 or base_data[-1]['timestamp'] - base_data[0]['timestamp'] < self.min_duration / 1000.:
                    self.recent_fixation = None
                    return

                dispersion, origin, base_data = gaze_dispersion(self.g_pool.capture, base_data, use_pupil)

                if dispersion < np.deg2rad(self.max_dispersion):
                    new_fixation = fixation_from_data(dispersion, origin, base_data)
                    events['fixations'].append(new_fixation)
                    self.recent_fixation = new_fixation
                else:
                    self.recent_fixation = None

                led_params = []

                # Check if gaze point lies with hit region
                for m in self.visible_markers_flipped:
                    hit_region = self.hit_region(m)
                    fs = self.g_pool.capture.frame_size  # frame height
                    x, y = None, None

                    if self.recent_fixation:
                        x, y = denormalize(self.recent_fixation['norm_pos'], fs, flip_y=True)
                        if cv2.pointPolygonTest(hit_region, (x, y), True) >= 0:
                            m['fixated'] = True
                    else:
                        pts = [denormalize(pt['norm_pos'], fs, flip_y=True) for pt in gaze]
                        if len(pts) > 0:
                            x, y = pts[0][0], pts[0][1]
                            if cv2.pointPolygonTest(hit_region, (x, y), True) >= 0:
                                m['within_roi'] = True

                    if x and y:
                        distance, angle = self.distance_and_angle(hit_region, x, y)
                        # logger.error("{}, {}".format(distance, angle))
                        m['distance'] = distance
                        m['angle'] = angle
                        # m['marker_angle'] = self.marker_angle(m)

                        # logger.error("Marker {}: {}".format(m['id'], m['marker_angle']))

                if len(gaze) > 0 and self.serial:
                    led_string = self.led_control.get_led_string(
                        self.visible_markers_flipped,
                        self.max_fiducual_id,
                        self.indicator_max_distance_to_marker,
                        self.angle_adjustment)

                    self.serial.write(led_string.encode())
                elif self.serial:
                    # Clear LEDs
                    led_string = self.led_control.clear()
                    self.serial.write(led_string.encode())

            # Call Ryan's code
            self.ryans_code(self.visible_markers_flipped)

    def ryans_code(self, markers):
        # RYAN: INSERT CODE HERE

        # Loop through all markers in the screen
        for m in markers:
            # If this marker is fixated on
            if m['fixated']:
                marker_id = m['id'] # get the marker id number e.g. 4 or 5
                # do something

    def hit_region(self, m):
        self.hit_region_percentage = ((self.hit_region_extension_percentage / 100.0) / 2.0)
        pt = 1.0 + self.hit_region_percentage
        point_array = [[[1 - pt, 1 - pt], [1 - pt, pt], [pt, pt], [pt, 1 - pt]]]
        hit_region_frame = np.array(point_array, dtype=np.float32)
        return cv2.perspectiveTransform(hit_region_frame, m_marker_to_screen(m))

    def marker_angle(self, m):
        mRoi = cv2.moments(hit_region)
        xRoi = int(mRoi["m10"] / mRoi["m00"])
        yRoi = int(mRoi["m01"] / mRoi["m00"])

        angle = np.arctan2((y - yRoi),(x - xRoi)) * 180 / np.pi
        if angle < 0:
            angle = angle + 360

        return angle

    def distance_and_angle(self, hit_region, x, y):
        mRoi = cv2.moments(hit_region)
        xRoi = int(mRoi["m10"] / mRoi["m00"])
        yRoi = int(mRoi["m01"] / mRoi["m00"])

        distance = np.linalg.norm(np.array((xRoi, yRoi)) - np.array((x, y)))
        angle = np.arctan2((y - yRoi),(x - xRoi)) * 180 / np.pi
        if angle < 0:
            angle = angle + 360

        return distance, angle

    def clear_leds(self):
        if self.serial:
            # Clear LEDs
            led_string = self.led_control.clear()
            self.serial.write(led_string.encode())

    def gl_display(self):
        if self.running:
            for m in self.visible_markers_flipped:
                marker_color = RGBA(1., .1, .2, .3)  # red
                if m['fixated']:
                    marker_color = RGBA(.1, 1., .2, .3)  # green

                point_count = 4
                point_array = [[[0, 0], [0, 1], [1, 1], [1, 0]]]

                # Show the direction of the marker with an arrow
                if self.show_marker_direction:
                    point_count = 6
                    point_array = [[[0, 0], [0, 1], [.5, 1.3], [1, 1], [1, 0], [0, 0]]]

                marker_frame = np.array(point_array, dtype=np.float32)
                marker_frame = cv2.perspectiveTransform(marker_frame, m_marker_to_screen(m))
                reshape = marker_frame.reshape((point_count, 2))
                centroid = m['centroid']

                # Draw marker
                if m['perimeter'] >= self.min_marker_perimeter and m['id_confidence'] > self.min_id_confidence:
                    draw_polyline(reshape, color=marker_color)
                    draw_polyline(reshape, color=marker_color, line_type=GL_POLYGON)
                else:
                    draw_polyline(reshape, color=marker_color)

                # Draw the extended hit region
                if self.hit_region_percentage > 0.0:
                    pt = 1.0 + self.hit_region_percentage
                    point_array = [[[1 - pt, 1 - pt], [1 - pt, pt], [pt, pt], [pt, 1 - pt]]]

                    hit_region_frame = np.array(point_array, dtype=np.float32)
                    hit_region_frame = cv2.perspectiveTransform(hit_region_frame, m_marker_to_screen(m))
                    hit_region_reshape = hit_region_frame.reshape((4, 2))

                    hit_region_color = marker_color
                    hit_region_color.a = 0.3

                    draw_polyline(hit_region_reshape, color=hit_region_color)
                    draw_polyline(hit_region_reshape, color=hit_region_color, line_type=GL_POLYGON)

                if self.show_marker_names:
                    self.glfont.draw_text(centroid[0] - 3, centroid[1] + 6, str(m['id']))


    def cleanup(self):
        self.clear_leds()
