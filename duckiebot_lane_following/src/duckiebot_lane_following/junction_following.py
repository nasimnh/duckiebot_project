#!/usr/bin/env python

import numpy as np

import cv2

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from particle_filter import ParticleFilter
import lane_follower

import functions

class JunctionFollower:
    def __init__(self):
        
	lane_following = lane_follower.LaneFollower()
        # Publisers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	self.direction_pub = rospy.Publisher('/junction_direction', String, queue_size=1)

        # Image subscriber
        #self.image_sub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)
	self.bound_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_callback)

    def bounding_callback(self, bounding_msg):
	if bounding_msg.bounding_boxes[0].xmin in range(380, 505) and bounding_msg.bounding_boxes[1].xmin in range(0, 100):
	    lane_follower.LaneFollower().stop
	    direction = "Go straight"
	    self.direction_pub.publish(direction)
	if bounding_msg.bounding_boxes[1].xmin in range(380, 505) and bounding_msg.bounding_boxes[0].xmin in range(0, 100):
	    lane_follower.LaneFollower().stop
	    direction = "Go straight"
	    self.direction_pub.publish(direction)
	if bounding_msg.bounding_boxes[1].xmin in range(380, 505) and bounding_msg.bounding_boxes[0].xmin in range(225, 365):
	    lane_follower.LaneFollower().stop
	    direction = "Go left"
	    self.direction_pub.publish(direction)
	if bounding_msg.bounding_boxes[0].xmin in range(380, 505) and bounding_msg.bounding_boxes[1].xmin in range(225, 365):
	    lane_follower.LaneFollower().stop
	    direction = "Go left"
	    self.direction_pub.publish(direction)
	if bounding_msg.bounding_boxes[1].xmin in range(0, 100) and bounding_msg.bounding_boxes[0].xmin in range(225, 365):
	    lane_follower.LaneFollower().stop
	    direction = "Go right"
	    self.direction_pub.publish(direction)
	if bounding_msg.bounding_boxes[0].xmin in range(0, 100) and bounding_msg.bounding_boxes[1].xmin in range(225, 365):
	    lane_follower.LaneFollower().stop
	    direction = "Go right"
	    self.direction_pub.publish(direction)

    def stop(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('junction_follower')

    junction_follower = JunctionFollower()

    rospy.on_shutdown(junction_follower.stop)

    rospy.spin()
