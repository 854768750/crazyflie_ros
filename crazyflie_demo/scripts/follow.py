#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped

class Follow():
    def __init__(self, goals):
        rospy.init_node('follow', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.goal = rospy.get_param("~goal")
        self.x = rospy.get_param("~x")
        self.y = rospy.get_param("~y")
        self.z = rospy.get_param("~z")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.listener = TransformListener()
        self.goals = goals
        self.goalIndex = 0

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        while not rospy.is_shutdown():
            goal.header.seq += 1
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = self.x
            goal.pose.position.y = self.y
            goal.pose.position.z = self.z
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]

            self.pubGoal.publish(goal)

            t = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
            if self.listener.canTransform(self.worldFrame, self.frame, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
                rpy = tf.transformations.euler_from_quaternion(quaternion)
		#rospy.loginfo(rpy)
                if     math.fabs(position[0] - self.x) < 0.25 \
                   and math.fabs(position[1] - self.y) < 0.25 \
                   and math.fabs(position[2] - self.z) < 0.25 \
                   and math.fabs(rpy[2] - 0) < math.radians(10):
                        rospy.sleep(3)
                        self.goalIndex += 1
			break

	while not rospy.is_shutdown():
	    goal.header.seq += 1
	    goal.header.stamp = rospy.Time.now()
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]

	    t = self.listener.getLatestCommonTime(self.worldFrame, self.goal)
            if self.listener.canTransform(self.worldFrame, self.goal, t):
                position, quaternion = self.listener.lookupTransform(self.worldFrame, self.goal, t)
		goal.pose.position.x = position[0]-0.8*math.sin(rpy[2])
            	goal.pose.position.y = position[1]+0.8*math.cos(rpy[2])
            	goal.pose.position.z = position[2]+0.5
		rpy = tf.transformations.euler_from_quaternion(quaternion)
		#rospy.loginfo(rpy)
	    self.pubGoal.publish(goal)





