#!/usr/bin/env python

import argparse
import cv2
import RPi.GPIO as GPIO
import os
import os.path
from pushbullet import Pushbullet

def capture(video_cap_dev, video_path, length_secs, xres, yres, framerate):
    count = 1
    frames = length_secs * framerate

    print "Setting up OpenCV"
    capture = cv2.VideoCapture(video_cap_dev)
    format = cv2.cv.CV_FOURCC(*'XVID')

    print "Capturing Video"
    video_writer = cv2.VideoWriter(video_path, format, framerate, (xres, yres))
    while (capture.isOpened()):
        ret, frame = capture.read()
        if ret:
            video_writer.write(frame)
        else:
            break

        count += 1
        if count > frames:
            break

    print "Cleaning up OpenCV"
    capture.release()
    cv2.destroyAllWindows()

    print "OpenCV Done!"

def send(pushbullet, video_path, msg):
    print "Opening Video"
    with open(video_path, "rb") as data:
        print "Uploading Video"
        file_data = pushbullet.upload_file(data, msg)

    print "Sending Video"
    push = pushbullet.push_file(**file_data)

    print "Pushbullet Done!"

def on_run(args):
    gpio_active_state = (GPIO.HIGH if args.gpio_active == True else GPIO.LOW)
    gpio_active_px = (GPIO.PUD_DOWN if args.gpio_active == True else GPIO.PUD_UP)
    gpio_channel = args.gpio_num
    video_cap_dev = args.cap_device
    video_xres = args.xres
    video_yres = args.yres
    video_rate = args.framerate
    video_length = args.video_length
    video_path = args.video_path
    frames = args.framerate
    api_key = args.api_key
    video_name = args.video_name


    if not os.path.exists(video_path):
        os.makedirs(video_path)
    elif not os.path.isdir(video_path):
        print "video_path must be a directory!"
        exit(1)

    video_path = os.path.join(video_path, "output.avi")

    print "Initializing Pushbullet"
    pb = Pushbullet(api_key)

    print "Initializing GPIO"
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio_channel, GPIO.IN, pull_up_down=gpio_active_px)
    print gpio_active_state

    print "Running"
    try:
        while True:
            if GPIO.input(gpio_channel) == gpio_active_state:
                print "Snapping Movie!"
                capture(video_cap_dev, video_path, video_length, video_xres, video_yres, video_rate)
                send(pb, video_path, video_name)

    except KeyboardInterrupt:
        print "Quitting"


    print "Cleaning up GPIO's"
    GPIO.cleanup()

parser = argparse.ArgumentParser(description="Captures a video from a webcam when a GPIO is triggered on the Raspberry PI.\n"
                                             "Then sends the video to the user via Pushbullet.")
parser.add_argument('-video_length', help='Length of video to create in seconds.', type=int, default=10, required=False)
parser.add_argument('-framerate', help='Framerate to capture at.', type=int, default=20, required=False)
parser.add_argument('-xres', help='X resolution to capture at.', type=int, default=640, required=False)
parser.add_argument('-yres', help='Y resolution to capture at.', type=int, default=480, required=False)
parser.add_argument('-cap_device', help='Capture device index, if only one camera then you probably dont need to specify.', type=int, default=0, required=False)
parser.add_argument('-api_key', help='Pushbullet API key.', required=True)
parser.add_argument('-gpio_num', help='GPIO channel (BCM mode) to wait for.', type=int, required=True)
parser.add_argument('-gpio_active_high', help='Active high for when the capture should begin.', dest='gpio_active', required=False, action='store_true')
parser.add_argument('-gpio_active_low', help='Active low for when the capture should begin.', dest='gpio_active', required=False, action='store_false')
parser.add_argument('-video_path', help='Local path to save videos.', required=True)
parser.add_argument('-video_name', help='Name to give video when sending to pushbullet.', required=True)
parser.set_defaults(gpio_active=False)
parser.set_defaults(func=on_run)
args = parser.parse_args()
args.func(args)
