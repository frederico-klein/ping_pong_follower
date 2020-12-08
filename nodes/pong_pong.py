#!/usr/bin/env python

import rospy
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def callback(msg):
    marker.pose.position.x = msg.z
    marker.pose.position.y = -msg.x
    marker.pose.position.z = -msg.y
    mypub.publish(marker)

rospy.init_node("pong_pong_marker_publisher", anonymous = False)

mypub = rospy.Publisher("pongo", Marker, queue_size=1)

rospy.Subscriber("topic", Point, callback)

marker = Marker()
marker.header.frame_id = "camera_link"
marker.header.stamp = rospy.Time.now()
marker.ns = "what"
marker.id = 0
marker.type = Marker.SPHERE
marker.action = Marker.ADD
marker.pose.position.x = 0.04
marker.pose.position.y = 0.04
marker.pose.position.z = 0.04
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
marker.scale.x = 0.04
marker.scale.y = 0.04
marker.scale.z = 0.04
marker.color.a = 0.9
marker.color.r = 0.8
marker.color.g = 0.5
marker.color.b = 0.2

rospy.spin()
