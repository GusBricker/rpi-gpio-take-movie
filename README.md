# rpi-gpio-take-movie
For use on the Raspberry PI to take a movie when a GPIO signals to do so.

On GPIO entering its active state, uses OpenCV to capture from a video capture device (in this case a PS3 Eyetoy).
Then sends the video to an end user via Pushbullet.


### Dependancies:
`apt-get install python-opencv` - Installs OpenCV and its Python bindings.
`pip install RPi.GPIO` - Provides GPIO Python API.
`pip install pushbullet.py` - Provides Pushbullet Python API.


### License:
MIT - Have fun!
