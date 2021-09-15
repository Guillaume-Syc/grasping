#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import sys

from visualization_msgs.msg import Marker, MarkerArray
from grasp_generator.msg import gripper

# Variable globales
pub_makers = rospy.Publisher('visualization_marker', Marker, queue_size=10)


def delete_all_markers(data):

    marker = Marker()
    marker.header.frame_id = data.header.frame_id
    marker.action = 3 # deleteall
    pub_makers.publish(marker)

def callback_grasp(data):
    if (data.number == 1):
        delete_all_markers(data)
    marker = Marker()
    marker.ns = "gripper"
    marker.id = data.number
    marker.header.frame_id = data.header.frame_id
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    if(data.score >= 0.8):
        marker.color.r = 1.0
        marker.color.g = 0.0
    marker.pose.orientation = data.pose.orientation
    marker.pose.position = data.pose.position
    marker.mesh_resource = "file:///home/coro-lambda/Documents/Guillaume/public_sim_ws/src/grasping/grasp_generator/scripts/gripper.STL"
    pub_makers.publish(marker)


def listener():
    rospy.init_node('vizualise_grasp_rviz', anonymous=True)
    rospy.Subscriber("/grasp", gripper, callback_grasp)
    rospy.spin()


if __name__ == '__main__':
    listener()

