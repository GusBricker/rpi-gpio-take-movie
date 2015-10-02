#!/usr/bin/env python

import argparse
import cv2
import time
from cv2 import *
import RPi.GPIO as GPIO
import os
import os.path
from pushbullet import Pushbullet

# Credit: https://github.com/jrosebr1/imutils/blob/b7e36aad0e8240d5ef39a400d4df78253c1f90b8/imutils/convenience.py#L41
# Copied this function as the library doesn't work properly with an older version of OpenCV (<2.4.5)
def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation=inter)

    # return the resized image
    return resized

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
    api_key = args.api_key
    video_name = args.video_name
    min_area = args.min_area
    detect_size = args.detect_size
    threshold = args.threshold
    blur = args.blur
    rebase_period = args.rebase_period
    num_frames = video_rate * video_length


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

    print "Initializing OpenCV"
    base_frame = None
    detected = False
    count = 1
    capture = cv2.VideoCapture(video_cap_dev)
    format = cv2.cv.CV_FOURCC(*'XVID')
    last_rebase_timestamp = time.time()

    print "Running"
    try:
        while True:
            (grabbed, frame) = capture.read()

            # If the frame could not be grabbed, then we have reached the end
            # of the video
            if not grabbed:
                break

            # Resize the frame, convert it to grayscale, and blur it
            frame = resize(frame, width=detect_size)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (blur, blur), 0)

            # Base frame
            time_diff = time.time() - last_rebase_timestamp
            if base_frame is None or (detected == False and time_diff > rebase_period):
                base_frame = gray
                last_rebase_timestamp = time.time()
                print "Getting a new base frame"
                continue

            # Compute the absolute difference between the current frame and
            # first frame
            frame_delta = cv2.absdiff(base_frame, gray)
            thresh = cv2.threshold(frame_delta, threshold, 255, cv2.THRESH_BINARY)[1]

            # Dilate the thresholded image to fill in holes, then find contours
            # on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=2)
            (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)

            # Loop over the contours
            for c in cnts:
                # If the contour is too small, ignore it
                if cv2.contourArea(c) < min_area:
                    continue

                # Compute the bounding box for the contour, draw it on the frame,
                # and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Detected object on the camera
                if not detected:
                    count = 1
                    detected = True
                    print "Object Detected!"
                    video_writer = cv2.VideoWriter(video_path, format, video_rate, (video_xres, video_yres))

            # Detected via gpio activation
            if not detected:
                if GPIO.input(gpio_channel) == gpio_active_state:
                    print "GPIO Activated!"
                    count = 1
                    detected = True
                    video_writer = cv2.VideoWriter(video_path, format, video_rate, (video_xres, video_yres))

            # Once detected, record into a video stream and send via Pushbullet
            if detected == True:
                if capture.isOpened():
                    video_writer.write(frame)

                    count += 1
                    if count > num_frames:
                        detected = False

                        video_writer = None

                        send(pb, video_path, video_name)

    except KeyboardInterrupt:
        print "Quitting"


    print "Cleaning up GPIO's"
    GPIO.cleanup()

    print "Cleaning up OpenCV"
    capture.release()
    cv2.destroyAllWindows()

parser = argparse.ArgumentParser(description="Captures a video from a webcam when a person is detected in the camera's vision or\n"
                                             "a GPIO is triggered on the Raspberry PI.\n"
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
parser.add_argument('-min_area', help='Minimum area to detect.', type=int, required=True)
parser.add_argument('-detect_size', help='Size to shrink frames down before running comparing them. This is a speed optimization.', type=int, required=True)
parser.add_argument('-threshold', help='Threshold to a person.', type=int, required=True)
parser.add_argument('-blur', help='Size of blur to apply to each frame.', type=int, required=True)
parser.add_argument('-rebase_period', help='Frequency in seconds to update our base frame (providing the system isnt in the middle of a detection).', type=int, required=True)
parser.set_defaults(gpio_active=False)
parser.set_defaults(func=on_run)
args = parser.parse_args()
args.func(args)
