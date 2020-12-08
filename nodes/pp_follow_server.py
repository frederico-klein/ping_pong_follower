#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

from dynamic_reconfigure.server import Server
from ping_pong_follower.cfg import HSV_thresholdingConfig

max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value

def callback(config, level):
    global low_H,low_S,low_V,high_H,high_S,high_V
        # global low_H
        # global low_S
        # global low_V
        # global high_H
        # global high_S
        # global high_V
    rospy.loginfo("""Reconfigure Request: H {low_H_param}, {high_H_param},\
          S {low_S_param}, {high_S_param},\
          V {low_V_param}, {high_V_param},\
          """.format(**config))

    low_H     = config['low_H_param']
    high_H    = config['high_H_param']
    low_S     = config['low_S_param']
    high_S    = config['high_S_param']
    low_V     = config['low_V_param']
    high_V    = config['high_V_param']

    #print(config)
    #print(dir(config))
    print("H {}, {}".format(low_H, high_H))
    print("S {}, {}".format(low_S, high_S))
    print("V {}, {}".format(low_V, high_V))

    return config

class KeinBockMehr:

    def __init__(self, topic = "/camera/rgb/image_rect_color"):
        self.image_topic = topic

        self.bridge = CvBridge()

        rospy.Subscriber(self.image_topic, Image, self.image_callback)

        self.image_pub = rospy.Publisher("output_image", Image, queue_size=1)
        rospy.loginfo_once("i MUSt have a publisher")

    def image_callback(self, msg):
        #print("Received an image!")
        # global low_H
        # global low_S
        # global low_V
        # global high_H
        # global high_S
        # global high_V
        # low_H = 0
        # low_S = 0
        # low_V = 0
        # high_H = max_value_H
        # high_S = max_value
        # high_V = max_value
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            frame_HSV = cv.cvtColor(cv2_img, cv.COLOR_BGR2HSV)
            frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

        except CvBridgeError, e:
            print(e)
        else:

            msg = self.bridge.cv2_to_imgmsg(frame_threshold, encoding="mono8") ##its already timestamped. i dont need to do this!
            self.image_pub.publish(msg)
            rospy.loginfo_once("i MUSt have published something")


if __name__ == "__main__":
    rospy.init_node("ping_pong_follower", anonymous = False)

    srv = Server(HSV_thresholdingConfig, callback)
    mytHING = KeinBockMehr()
    rospy.spin()
