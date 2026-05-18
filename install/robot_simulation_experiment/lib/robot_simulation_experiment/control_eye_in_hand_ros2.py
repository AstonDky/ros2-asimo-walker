#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 version of control_eye_in_hand.py
Publishes:   /robot/target_joints     std_msgs/msg/Float64MultiArray
Subscribes: /robot/joints             std_msgs/msg/Float64MultiArray
            /robot/end_pose           std_msgs/msg/Float64MultiArray
            /kinect/depth             sensor_msgs/msg/PointCloud2
"""

import os
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# Allow importing utils.py if it is placed in the same directory as this script.
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

try:
    from utils import Jacob
except ImportError as exc:
    raise ImportError(
        "Cannot import Jacob from utils.py. Put utils.py in the same scripts/ "
        "directory as this file, or install it as a Python module."
    ) from exc


class EyeInHandController(Node):
    def __init__(self):
        super().__init__('eye_in_hand_controller')

        self.robot_joints = None
        self.robot_end_pose = None
        self.point_cloud = None
        self.target_pose = None
        self.target_ready = False

        self.depth_camera_frame_in_world = np.array([
            [-1.00, 0.0, 0.0, -0.01410000026226],
            [0.0, 0.0, 1.0, 0.22925010323524],
            [0.0, 1.0, 0.0, 0.0823894739151],
        ])

        self.target_pub = self.create_publisher(
            Float64MultiArray,
            '/robot/target_joints',
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            '/robot/joints',
            self.update_robot_joints,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            '/robot/end_pose',
            self.update_robot_end_pose,
            10,
        )

        self.create_subscription(
            PointCloud2,
            '/kinect/depth',
            self.update_point_clouds,
            10,
        )

        self.frequency = 10.0
        self.dt = 1.0 / self.frequency
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('eye_in_hand_controller started.')
        self.get_logger().info('Waiting for /robot/joints, /robot/end_pose and /kinect/depth ...')

    def update_robot_joints(self, msg: Float64MultiArray):
        self.robot_joints = np.array(msg.data, dtype=float)

    def update_robot_end_pose(self, msg: Float64MultiArray):
        self.robot_end_pose = np.array(msg.data, dtype=float)

    def update_point_clouds(self, msg: PointCloud2):
        # ROS 2 replacement for sensor_msgs.point_cloud2.read_points_list.
        points = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points.append([float(p[0]), float(p[1]), float(p[2])])
        self.point_cloud = np.array(points, dtype=float) if points else None

    def find_point_center(self):
        if self.point_cloud is None or self.point_cloud.size == 0:
            return None

        pc = np.concatenate(
            (self.point_cloud, np.ones((self.point_cloud.shape[0], 1))),
            axis=1,
        )
        pc_in_world = np.matmul(self.depth_camera_frame_in_world, pc.T)

        valid = np.where((pc_in_world[2, :] > 0.001) & (pc_in_world[1, :] < 1.0))[0]
        if valid.size == 0:
            return None

        center = np.array([
            np.mean(pc_in_world[0, valid]),
            np.mean(pc_in_world[1, valid]),
            np.mean(pc_in_world[2, valid]),
        ])
        return center

    def try_initialize_target_pose(self):
        center = self.find_point_center()
        if center is None:
            self.get_logger().info('Receiving point cloud data, but no valid target points yet ...')
            return False

        self.target_pose = np.array([center[0], center[1], center[2], 0.0, 0.0, 0.0], dtype=float)
        self.target_ready = True
        self.get_logger().info(f'Target pose is {self.target_pose}')
        return True

    def control_loop(self):
        if self.robot_joints is None:
            self.get_logger().info('Waiting for /robot/joints ...')
            return

        if self.robot_end_pose is None:
            self.get_logger().info('Waiting for /robot/end_pose ...')
            return

        if not self.target_ready:
            if self.point_cloud is None:
                self.get_logger().info('Waiting for /kinect/depth ...')
                return
            if not self.try_initialize_target_pose():
                return

        this_robot_joints = np.array(self.robot_joints, dtype=float)
        if this_robot_joints.size < 6:
            self.get_logger().warn(f'/robot/joints has {this_robot_joints.size} values, expected 6.')
            return

        try:
            jaco = Jacob(this_robot_joints)
            error = self.target_pose - self.robot_end_pose

            w_weight = np.array([
                [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.01],
            ])

            d_pose = np.matmul(np.linalg.pinv(jaco), self.dt * np.matmul(w_weight, error))

            if abs(np.linalg.det(jaco)) < 0.01:
                d_pose = np.array([-0.01, -0.01, -0.01, 0.01, 0.01, 0.01])

            new_joints = d_pose + this_robot_joints

            msg = Float64MultiArray()
            msg.data = new_joints.tolist()
            self.target_pub.publish(msg)

        except Exception as exc:
            self.get_logger().error(f'Control loop failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = EyeInHandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
