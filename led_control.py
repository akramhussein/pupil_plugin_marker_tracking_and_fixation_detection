#
#   Pupil Plugin - Marker Tracking and Fixation Detection
#
#   Copyright 2017 Shadi El Hajj
#

MINWIDTH = 1
MAXWIDTH = 6
MAX_DISTANCE = 1000
LEDS_PER_BOX = 28

import logging
logger = logging.getLogger(__name__)

class LedControl():

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def clear(self):
        return '0' * (LEDS_PER_BOX * 3) + '\n'

    def get_led_string(self, markers, max_marker_id, max_distance):
        string = ''
        for i in range(1, max_marker_id+1):
            ids = list(map(lambda x: x['id'], markers))
            if i in ids:
                m = next((marker for marker in markers if marker['id'] == i))
                string += self.get_led_param_per_fiducial(m, max_distance)
            else:
                string += '0'*(LEDS_PER_BOX*3)
        return string + '\n'

    def get_led_param_per_fiducial(self, marker, max_distance):
        string = ''
        if marker['fixated']:
            string += '030' * LEDS_PER_BOX
        elif marker['within_roi']:
            string += '300' * LEDS_PER_BOX
        else:
            distance = int(round(marker['distance']))
            angle = int(round(marker['angle'])) - (marker['rotation'] * 90)

            if distance == 0.0 or angle == 0.0:
                return '0' * (LEDS_PER_BOX * 3)

            dist = self.map(distance, 0, max_distance, MAXWIDTH, MINWIDTH)
            centre_led = self.map(angle+45, -180, 180, 0, LEDS_PER_BOX)

            # make the ramp one-sided
            one_sided_ramp = [3 * (i / float(dist)) for i in range(0, int(dist))]

            # make the ramp two sided
            two_sided_ramp = one_sided_ramp + list(reversed(one_sided_ramp))
            padded_ramp = two_sided_ramp + [0]*(LEDS_PER_BOX-len(two_sided_ramp))

            # centre the ramp around the right pixel and set the pixel colour
            for i in range(0, LEDS_PER_BOX):
                index = (centre_led - i + int(dist)) % (LEDS_PER_BOX)
                val = padded_ramp[int(index)]
                string += '0' + str(int(val)) + str(int(val))

        return string
