# Marker Tracking and Fixation Detector - Pupil Plugin

A Pupil Plugin for Pupil Capture that detects ArUco Markers and identifies when one is being focused on.

# Requirements

* [Pupil Labs headset](https://pupil-labs.com/pupil/).

# Installation

1. Install [Pupil Capture](https://github.com/pupil-labs/pupil/releases/)

2. Open Pupil Capture - this will create directory at `~/pupil_capture_settings/`

3. Copy the all the files to `~/pupil_capture_settings/plugins`.

4. Restart Pupil Capture

5. Click the `Plugin Manager` tab on the right and activate `Marker Tracker And Fixation Detector`

# Usage

* Press `m` to on the keyboard to start or stop the marker tracking.

* Any ArUco Markers that appear in the world view will appear red.

* Fixate on an ArUco Marker and it will go green.

# Creating an ArUco Marker

A script is provided to produce ArUco Markers.

You can generate a PNG of the marker as such:

```
$ python make_square_markers.py <id>

e.g.

$ python make_square_markers.py 4
```
