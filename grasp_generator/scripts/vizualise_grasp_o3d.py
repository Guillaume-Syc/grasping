#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy.point_cloud2 as pt

from grasp_generator.msg import gripper


import os
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image
import math
import torch
from graspnetAPI import GraspGroup

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(ROOT_DIR, '../src/graspnet/models'))
sys.path.append(os.path.join(ROOT_DIR, '../src/graspnet/dataset'))
sys.path.append(os.path.join(ROOT_DIR, '../src/graspnet/utils'))
sys.path.append(os.path.join(ROOT_DIR, '../src/utils'))

from graspnet import GraspNet, pred_decode
from graspnet_dataset import GraspNetDataset
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image
import timerit
from tf.transformations import euler_from_quaternion, quaternion_from_euler


vis = o3d.visualization.Visualizer()

def vis_grasps(gg, cloud):
    with timerit.TimeIt('Visualize'):
        gg.nms()
        gg.sort_by_score()
        # only take the 50th best grasping
        gg = gg[:50]
        grippers = gg.to_open3d_geometry_list()
        o3d.visualization.draw_geometries([cloud, *grippers])

def callback_cloud(data):
    cloud = pt.pointcloud2_to_xyz_array(data)
    # downgrade the sampling from 2 600 000 to 800 000
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    cloud = pcd.voxel_down_sample(voxel_size=0.0005)
    vis.add_geometry(cloud)
  


def callback_grasp(data) :
    data = 0

def listener () :
    vis.create_window()
    rospy.init_node('vizualise_grasp_o3d', anonymous=True)
    rospy.Subscriber("/points2", PointCloud2, callback_cloud)
    rospy.Subscriber("/grasp", gripper, callback_grasp)
    rospy.spin()


if __name__ == '__main__':
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    listener()

