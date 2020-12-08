#!/usr/bin/env python

import rospy

import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from dynamic_reconfigure.server import Server
from ping_pong_follower.cfg import BlobberConfig






class Somthing:

    def __init__(self, topic = "/output_image", do_publish = True):

        srv = Server(BlobberConfig, self.dynamic_reconfigure_callback)

        self.image_topic = topic
        params = cv2.SimpleBlobDetector_Params()
        params.blobColor = 255 # we want the white blobs

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1500
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        self.detector = cv2.SimpleBlobDetector_create(params)
        self.bridge = CvBridge()
        self.do_publish = do_publish

        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        if self.do_publish:
            self.image_pub = rospy.Publisher("blob_image", Image, queue_size=1)
            rospy.loginfo_once("i MUSt have a publisher")

    def dynamic_reconfigure_callback(self, config, level):

        rospy.loginfo("""Reconfigure Request:               {filterByArea}, {minArea},\
               {filterByCircularity}, {minCircularity},\
               {filterByConvexity}, {minConvexity},\
              {filterByInertia}, {minInertiaRatio},\
              """.format(**config))
        params = cv2.SimpleBlobDetector_Params()
        params.blobColor = 255 # we want the white blobs
        # Filter by Area.
        params.filterByArea = config['filterByArea']
        params.minArea = config['minArea']
        # Filter by Circularity
        params.filterByCircularity =  config['filterByCircularity']
        params.minCircularity = config['minCircularity']
        # Filter by Convexity
        params.filterByConvexity = config['filterByConvexity']
        params.minConvexity = config['minConvexity']
        # Filter by Inertia
        params.filterByInertia = config['filterByInertia']
        params.minInertiaRatio = config['minInertiaRatio']

        self.detector = cv2.SimpleBlobDetector_create(params)
        return config

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            keypoints = self.detector.detect(cv2_img)
            #if keypoints:
            #    print(keypoints)
            if self.do_publish:
                ##we can draw as well.
                im_with_keypoints = cv2.drawKeypoints(cv2_img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        except CvBridgeError, e:
            print(e)
        else:
            if self.do_publish:
                msg = self.bridge.cv2_to_imgmsg(im_with_keypoints, encoding="bgr8")
                self.image_pub.publish(msg)
                rospy.loginfo_once("i MUSt have published something")


if __name__ == "__main__":
    rospy.init_node("ping_pong_blober", anonymous = False)

    mytHING = Somthing()
    rospy.spin()
