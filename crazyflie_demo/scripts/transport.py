#!/usr/bin/env python

import numpy as np
import csv
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name","goal")
    r = rospy.get_param("~rate","10.0")
    x0 = rospy.get_param("~x0","0.0")
    y0 = rospy.get_param("~y0","0.0")
    z0 = rospy.get_param("~z0","0.0")
    csv_file = rospy.get_param("~csv_file")
    rate = rospy.Rate(r)
    routes = np.loadtxt(csv_file,skiprows=1,delimiter=',') #routes of all cfs
    ids = np.loadtxt(csv_file,str,skiprows=1,delimiter=',',usecols=[0]) #ids of all cfs
    pose_index = 1 
    pose_num = (routes.shape[1]-1)/6
    msgs = range(0,routes.shape[0])  #its elements are set to PoseStamped later
    pubs = range(0,routes.shape[0])  #its elements are set to Publisher later
    print msgs
    for i in range(1,routes.shape[0]):
        msgs[i] = PoseStamped()
        msgs[i].header.seq = 0
        msgs[i].header.stamp = rospy.Time.now()
        msgs[i].header.frame_id = worldFrame
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        msgs[i].pose.orientation.x = quaternion[0]
        msgs[i].pose.orientation.y = quaternion[1]
        msgs[i].pose.orientation.z = quaternion[2]
        msgs[i].pose.orientation.w = quaternion[3]
        pubs[i] = rospy.Publisher("crazyflie"+ids[i][0]+"/"+"goal", PoseStamped, queue_size=1)

    while (not rospy.is_shutdown()) and (pose_index<=pose_num):
        for i in range(1,routes.shape[0]):
            msgs[i].header.seq += 1
            msgs[i].header.stamp = rospy.Time.now()
            msgs[i].pose.position.x = routes[i][7*pose_index-6] + x0
            msgs[i].pose.position.y = routes[i][7*pose_index-5] + y0
            msgs[i].pose.position.z = routes[i][7*pose_index-4] + z0 + 0.5
            pubs[i].publish(msgs[i])
        rospy.sleep(routes[1][7*pose_index-3])
        
        t_start= rospy.Time.now().to_sec()
        t_now=t_start
        while ((not rospy.is_shutdown()) and (t_now-t_start<routes[1][7*pose_index-2])):
            msgs[i].header.seq += 1
            msgs[i].header.stamp = rospy.Time.now()
            msgs[i].pose.position.x = routes[i][7*pose_index-6] + x0
            msgs[i].pose.position.y = routes[i][7*pose_index-5] + y0
            msgs[i].pose.position.z = routes[i][7*pose_index-4] + z0 + 0.5*(1-(t_now-t_start)/routes[1][7*pose_index-2])
            pubs[i].publish(msgs[i])
            t_now= rospy.Time.now().to_sec()

        for i in range(1,routes.shape[0]):
            msgs[i].header.seq += 1
            msgs[i].header.stamp = rospy.Time.now()
            msgs[i].pose.position.x = routes[i][7*pose_index-6] + x0
            msgs[i].pose.position.y = routes[i][7*pose_index-5] + y0
            msgs[i].pose.position.z = routes[i][7*pose_index-4] + z0 
            pubs[i].publish(msgs[i])
        rospy.sleep(routes[1][7*pose_index-1])

       
        t_start= rospy.Time.now().to_sec()
        t_now=t_start
        while ((not rospy.is_shutdown()) and (t_now-t_start<routes[1][7*pose_index])):
            msgs[i].header.seq += 1
            msgs[i].header.stamp = rospy.Time.now()
            msgs[i].pose.position.x = routes[i][7*pose_index-6] + x0
            msgs[i].pose.position.y = routes[i][7*pose_index-5] + y0
            msgs[i].pose.position.z = routes[i][7*pose_index-4] + z0 + 0.5*(t_now-t_start)/routes[1][7*pose_index]
            pubs[i].publish(msgs[i])
            t_now= rospy.Time.now().to_sec()

        pose_index = pose_index % pose_num + 1
        rate.sleep()




