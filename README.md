# rpi-gpio-take-movie

For use on the Raspberry PI to take a movie when a GPIO signals to do so.


`rpi-gpio-take-movie.py`


On a GPIO pin entering its active state, uses OpenCV to record from a video capture device (tested with PS3 Eyetoy).
Then sends the video to an end user via Pushbullet.

`rpi-gpio-detect-take-movie.py`


Will record a video and send via Pushbullet on after either two conditions are met:.
- After either a GPIO pin enters its active state.
- OpenCV detects motion via the video capture device (tested with PS3 Eyetoy).

Credit for the motion detection code goes to [Pyimagesearch](http://www.pyimagesearch.com/2015/05/25/basic-motion-detection-and-tracking-with-python-and-opencv/).


### Dependancies:
-------------------------

`apt-get install python-opencv` - Installs OpenCV and its Python bindings.

`pip install RPi.GPIO` - Provides GPIO Python API.

`pip install pushbullet.py` - Provides Pushbullet Python API.


### License:
-------------------------

MIT - Have fun!
