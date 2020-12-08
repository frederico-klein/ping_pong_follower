#!/usr/bin/env python

import rospy
import numpy as np

##gets more complicated eh?
import message_filters
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo ##ideally I should be reading CameraInfo here
from cv_bridge import CvBridge, CvBridgeError
import cv2

from dynamic_reconfigure.server import Server
from ping_pong_follower.cfg import HSVandBlobConfig

# font
font = cv2.FONT_HERSHEY_SIMPLEX

def draw_pretty_points(canvas, keypoint_list, keypoints):
    for keypoint_text_raw, keypoint in zip(keypoint_list, keypoints):
        keypoint_text = np.asarray(keypoint_text_raw, dtype=int)
        #keypoint_text = np.asarray(keypoint_text_raw, dtype=np.uint8)
        canvas = cv2.putText(canvas, '{0}'.format(keypoint_text), (int(keypoint.pt[0]), int(keypoint.pt[1])), font,
                   0.5, (0,0,255), 1, cv2.LINE_AA)
    return canvas

def average_keypoint_value(canvas,keypoints):
    average_value = []
    if canvas.ndim == 2:
        nchannels = 1
    elif canvas.ndim > 2:
        nchannels = canvas.shape[-1]
    for keypoint in keypoints:
        circle_x =      int(keypoint.pt[0])
        circle_y =      int(keypoint.pt[1])
        circle_radius=  int(keypoint.size/2)
        #copypasta from https://stackoverflow.com/a/43170927/2594947
        circle_img = np.zeros((canvas.shape[:2]), np.uint8)
        cv2.circle(circle_img,(circle_x,circle_y),circle_radius,(255,255,255),-1)
        datos_rgb = cv2.mean(canvas, mask=circle_img)
        average_value.append(datos_rgb[:nchannels])
    return(average_value)

class ReallyKeinBockMehr:

    def __init__(self, topic_rgb = "/camera/rgb/image_rect_color", \
        topic_depth = "/camera/depth_registered/hw_registered/image_rect_raw", \
                #topic_depth = "/camera/depth/image_rect_raw", \ ###this is always there, but we want the registration with rgb!
        do_publish = False):

        self.srv = Server(HSVandBlobConfig, self.dynamic_reconfigure_callback)

        ###actually we don't need to start the values. on start dynamic_reconfigure issues a callback.

        self.image_topic = topic_rgb
        self.depth_topic = topic_depth

        self.bridge = CvBridge()
        self.do_publish = do_publish

        image_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        # ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10) ##maybe ApproximateTimeSynchronizer ?
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.combined_image_and_depth_callback)

        self.point_pub = rospy.Publisher("point_pos", Point, queue_size=1)
        ##I need to know the true resolution of the depth topic. here a wait_for_message would be the solution
        self.depth_shape = (640,480)
        self.depth_fov_angle = (70.0 /180*np.pi, 60.0 /180*np.pi)

        if self.do_publish:
            self.image_pub = rospy.Publisher("blob_image", Image, queue_size=1)
            self.depth_pub = rospy.Publisher("blob_depth", Image, queue_size=1)
            rospy.loginfo_once("I must have an image and depth publisher.")
        else:
            rospy.logwarn_once("Only publishing point positions.")  

    def dynamic_reconfigure_callback(self, config, level):

        rospy.logdebug("""Reconfigure Request: SimpleBlobDetector_Params
                      {filterByArea}, {minArea},\
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

        rospy.logdebug("""Reconfigure Request: H {low_H_param}, {high_H_param},\
              S {low_S_param}, {high_S_param},\
              V {low_V_param}, {high_V_param},\
              """.format(**config))
        self.low_H     = config['low_H_param']
        self.high_H    = config['high_H_param']
        self.low_S     = config['low_S_param']
        self.high_S    = config['high_S_param']
        self.low_V     = config['low_V_param']
        self.high_V    = config['high_V_param']

        return config

    def combined_image_and_depth_callback(self, img_msg, depth_msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "rgb8")
            cv2_dep16 = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough") ##erm its 16 bit though, isnt it?

            frame_HSV = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
            frame_threshold = cv2.inRange(frame_HSV, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))
            keypoints = self.detector.detect(frame_threshold)

            ##we want some colormap for the depth, so we can put a red circle around it
            ##we need to lose some of our resolution to show it
            cv2_dep8 = cv2_dep16.astype(np.uint8)
            frame_depth_mapped = cv2.applyColorMap(cv2_dep8, cv2.COLORMAP_JET)

            if keypoints:
                #print(keypoints) ## list!
                #print(dir(keypoints[0])) ## list!
                #print((keypoints[0].angle)) ## always -1?
                #print((keypoints[0].pt)) ## tuple x,y, in float
                dkpts = average_keypoint_value(cv2_dep16,keypoints)
                rospy.logdebug("depth averages[mm]: {0}".format(str(dkpts))) ##let's use full resolution here
                ckpts = average_keypoint_value(frame_HSV,keypoints)
                rospy.logdebug("HSV averages: {0}".format(str(ckpts)))
                rospy.logdebug("keypoint[0] cam pos: (%d,%d)"%(keypoints[0].pt[0],keypoints[0].pt[1]))
                #from the docs, the angle of view of the kinnect for windows is 70 and 60 degrees respectively.
                #an object at the edge of the screen is thus at +-35,+-30
                r = float(dkpts[0][0]) ### already in mm, supposedly
                x = np.sin((keypoints[0].pt[0]/float(self.depth_shape[0])-0.5)*self.depth_fov_angle[0])*r
                y = np.sin((keypoints[0].pt[1]/float(self.depth_shape[1])-0.5)*self.depth_fov_angle[1])*r
                rospy.logdebug("x: %f, y: %f, r %f"%(x,y,r))
                # x = np.sin(float(keypoints[0].pt[0]-self.depth_shape[0]/2)/(self.depth_shape[0]/2)*35)*z
                # y = np.sin(float(keypoints[0].pt[1]-self.depth_shape[1]/2)/(self.depth_shape[1]/2)*30)*z

                point_msg = Point()
                point_msg.x = x/1000.0 ## make it metric
                point_msg.y = y/1000.0
                point_msg.z = np.sqrt(r**2-x**2-y**2)/1000.0

                #print("%s, size: %f"%(str(keypoints[0].pt),keypoints[0].size))
            if self.do_publish:
                ##we can draw as well.
                im_with_keypoints = cv2.drawKeypoints(frame_threshold, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                dp_with_keypoints = cv2.drawKeypoints(frame_depth_mapped, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                if keypoints:
                    im_with_keypoints = draw_pretty_points(im_with_keypoints, ckpts, keypoints)
                    dp_with_keypoints = draw_pretty_points(dp_with_keypoints, dkpts, keypoints)

        except CvBridgeError, e:
            print(e)
        else:
            if keypoints:
                self.point_pub.publish(point_msg)
                rospy.logwarn_once("I must have published at least one keypoint.")
            if self.do_publish:
                imgmsg = self.bridge.cv2_to_imgmsg(im_with_keypoints, encoding="bgr8") ##its already timestamped. i dont need to do this!
                self.image_pub.publish(imgmsg)
                depmsg = self.bridge.cv2_to_imgmsg(dp_with_keypoints, encoding="bgr8") ##its already timestamped. i dont need to do this!
                self.depth_pub.publish(depmsg)
                rospy.loginfo_once("I must have published something.")


if __name__ == "__main__":
    rospy.init_node("ping_pong_follower", anonymous = False)
    
   
    if rospy.has_param('~publish_images'):
    	doIPublish = rospy.get_param('~publish_images')
    else:
        doIPublish = True
        rospy.logwarning("Publish_images not set. assuming true, will publish")
    
    mytHING = ReallyKeinBockMehr(do_publish = doIPublish)
    rospy.spin()
