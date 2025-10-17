#!/usr/bin/env python
import socket
import time

import imagezmq
import imutils
import rospy


def init_server():
    sender = imagezmq.ImageSender(connect_to="tcp://192.168.1.238:5555")
    rpiName = socket.gethostname()
    #vs = VideoStream().start()
    vs = imutils.Video.VideoStream(src=0).start()
    time.sleep(2.0)
    print("Camera ready")
    return sender, rpiName, vs

def start_stream(sender, rpiName, vs):
    while True:
    	# read the frame from the camera and send it to the server
        frame = vs.read()
        frame = imutils.resize(frame, width=320)
        sender.send_image(rpiName, frame)
        

if __name__ == "__main__":
    try:
        rospy.init_node("camera_stream", anonymous=True)
        start_stream(init_server())
    except rospy.ROSInterruptException:
        pass