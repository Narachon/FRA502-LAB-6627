#!/usr/bin/env python3
"""
Random Pose Publisher Node for LAB4
Generates random reachable poses within workspace
Uses FK directly from URDF transforms
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import random
from math import pi, sin, cos
from interface.srv import RandomData

class RandomPosePublisher(Node):
    def __init__(self):
        super().__init__('random_pose_node')
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        self.srv = self.create_service(RandomData, "random_pose_server", self.random_service_callback)
        
        # Workspace limits
        self.r_max = 0.53
        self.r_min = 0.03
        self.z_min = 0.05
        self.singularity_threshold = 0.005
        
        self.get_logger().info("=== Random Pose Publisher Node Started ===")

    # ==================== TRANSFORMATION MATRICES ====================
    def Rz(self, theta):
        c, s = cos(theta), sin(theta)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    
    def Rx(self, theta):
        c, s = cos(theta), sin(theta)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
    
    def Trans(self, x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

    # ==================== FORWARD KINEMATICS (from URDF) ====================
    def forward_kinematic(self, q):
        q1, q2, q3 = q[0], q[1], q[2]
        
        T_0_1 = self.Trans(0, 0, 0.2) @ self.Rz(q1)
        T_1_2 = self.Trans(0, -0.12, 0) @ self.Rx(-pi/2) @ self.Rz(q2)
        T_2_3 = self.Trans(0, -0.25, 0.1) @ self.Rz(q3)
        T_3_ee = self.Trans(0, -0.28, 0) @ self.Rx(pi/2)
        
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
        
        return T_0_ee[:3, 3]

    # ==================== JACOBIAN ====================
    def compute_jacobian(self, q):
        delta = 0.0001
        J = np.zeros((3, 3))
        pos0 = self.forward_kinematic(q)
        
        for i in range(3):
            q_temp = q.copy()
            q_temp[i] += delta
            pos1 = self.forward_kinematic(q_temp)
            J[:, i] = (pos1 - pos0) / delta
        
        return J

    # ==================== VALIDATION ====================
    def is_valid_pose(self, q):
        """Check if pose is valid: in workspace, above ground, not singularity"""
        pos = self.forward_kinematic(q)
        x, y, z = pos[0], pos[1], pos[2]
        
        # 1. Check Z limit (above link_0)
        if z < self.z_min:
            return False, f"Z too low: {z:.3f} < {self.z_min}"
        
        # 2. Check workspace (distance from shoulder at z=0.2)
        dist = np.linalg.norm(pos - np.array([0, 0, 0.2]))
        if dist > self.r_max:
            return False, f"Out of reach: {dist:.3f} > {self.r_max}"
        if dist < self.r_min:
            return False, f"Too close: {dist:.3f} < {self.r_min}"
        
        # 3. Check singularity
        J = self.compute_jacobian(q)
        det_J = np.linalg.det(J)
        if np.abs(det_J) < self.singularity_threshold:
            return False, f"Singularity: det(J)={det_J:.5f}"
        
        return True, "OK"

    # ==================== SERVICE CALLBACK ====================
    def random_service_callback(self, request, response):
        self.get_logger().info('Generating random valid pose...')
        
        max_attempts = 100
        attempt = 0
        
        while attempt < max_attempts:
            attempt += 1
            
            # Random joint angles
            q1 = random.uniform(-pi/2, pi/2)    # Base rotation
            q2 = random.uniform(-0.5, 0.5)      # Shoulder
            q3 = random.uniform(0.2, 1.3)       # Elbow (safer range)
            
            q_rand = np.array([q1, q2, q3])
            
            # Validate
            is_valid, reason = self.is_valid_pose(q_rand)
            
            if is_valid:
                pos = self.forward_kinematic(q_rand)
                x, y, z = pos[0], pos[1], pos[2]
                
                self.get_logger().info(f"Valid pose found (attempt {attempt})")
                self.get_logger().info(f"  Joints: [{q1:.2f}, {q2:.2f}, {q3:.2f}]")
                self.get_logger().info(f"  Position: ({x:.3f}, {y:.3f}, {z:.3f})")
                
                # Send joint angles back
                response.inprogress = True
                response.position.x = float(q1)
                response.position.y = float(q2)
                response.position.z = float(q3)
                
                # Publish target in RViz
                self.publish_target(x, y, z)
                
                return response
        
        # Failed to find valid pose
        self.get_logger().warn(f"Failed to find valid pose after {max_attempts} attempts!")
        response.inprogress = False
        response.position.x = 0.0
        response.position.y = 0.0
        response.position.z = 0.0
        
        return response

    def publish_target(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_0"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()