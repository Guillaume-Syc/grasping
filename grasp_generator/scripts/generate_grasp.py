#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy.point_cloud2 as pt
np.set_printoptions(threshold=sys.maxsize)
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

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path',  help='Model checkpoint path', default=os.path.join(ROOT_DIR, '../src/graspnet/logs/log_kn/checkpoint.tar')) #required=True,
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
cfgs = parser.parse_args()

np.set_printoptions(threshold=sys.maxsize)

pub_grasp = rospy.Publisher('grasp', gripper, queue_size=10)


def callback(data):
    with timerit.TimeIt('Pre-compute'):
        # convert the point cloud to array

        cloud = pt.pointcloud2_to_xyz_array(data)
        # downgrade the sampling from 2 600 000 to 800 000
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        cloud = pcd.voxel_down_sample(voxel_size=0.0005)
        # convert point cloud to tensor
        end_points = dict()
        cloud_toch = torch.from_numpy(np.asarray(cloud.points)[np.newaxis].astype(np.float32))
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_toch = cloud_toch.to(device)
        end_points['point_clouds'] = cloud_toch

    demo(end_points, cloud)

def get_net():
    with timerit.TimeIt('Launch Model '):
        # Init the model
        net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
                cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        net.to(device)
        # Load checkpoint
        checkpoint = torch.load(cfgs.checkpoint_path)
        net.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint['epoch']
        print("-> loaded checkpoint %s (epoch: %d)"%(cfgs.checkpoint_path, start_epoch))
        # set model to eval mode
        net.eval()
    return net


def get_grasps(net, end_points):
    with timerit.TimeIt('Generate Grasping'):
        # Forward pass
        with torch.no_grad():
            end_points = net(end_points)
            grasp_preds = pred_decode(end_points)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(gg_array)
    return gg

def collision_detection(gg, cloud):
    with timerit.TimeIt('Collision Check'):
        mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
        collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
        gg = gg[~collision_mask]
    return gg

def vis_grasps(gg, cloud):
    with timerit.TimeIt('Visualize'):
        gg.nms()
        gg.sort_by_score()
        # only take the 50th best grasping
        gg = gg[:50]
        grippers = gg.to_open3d_geometry_list()
        o3d.visualization.draw_geometries([cloud, *grippers])


def demo(end_points, cloud):
    # Graspnet need the point cloud as a tensor into a dict
    net = get_net()
    gg = get_grasps(net, end_points)

    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.asarray(cloud.points))
    #print(gg)

    # vis_grasps(gg, cloud)
    send_grasp(gg)

def send_grasp(gg):
    for grasp in gg:
        msg = gripper()
        msg.header.frame_id = "rgb_camera_link"

        msg.pose.position.x = grasp.translation[0]

        msg.pose.position.y = grasp.translation[1]
        msg.pose.position.z = grasp.translation[2]
        msg.pose.orientation.w = math.sqrt((1 + grasp.rotation_matrix[0][0] + grasp.rotation_matrix[1][1] + grasp.rotation_matrix[2][2])/2)
        msg.pose.orientation.x = (grasp.rotation_matrix[2][1] - grasp.rotation_matrix[1][2]) / 4 * msg.pose.orientation.w
        msg.pose.orientation.y = (grasp.rotation_matrix[0][2] - grasp.rotation_matrix[2][0]) / 4 * msg.pose.orientation.w
        msg.pose.orientation.z = (grasp.rotation_matrix[1][0] - grasp.rotation_matrix[0][1]) / 4 * msg.pose.orientation.w
        msg.score = grasp.score
        pub_grasp.publish(msg)




def listener():
    rospy.init_node('save_img', anonymous=True)
    rospy.Subscriber("/points2", PointCloud2, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
