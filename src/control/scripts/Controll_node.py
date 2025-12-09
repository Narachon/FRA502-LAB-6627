#!/usr/bin/python3
"""
Controller Node for LAB4: 3R Kinematics
Modes: IK (IPK), TO, AT (AM)
FK/IK calculated directly from URDF transforms
"""

import rclpy
from rclpy.node import Node
from interface.srv import ControllerData
from interface.srv import RandomData
import numpy as np
from math import pi, sin, cos
import time
from scipy.optimize import minimize

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

class Controller(Node):
    def __init__(self):
        super().__init__('Control')
        
        # === Services ===
        self.srv = self.create_service(ControllerData, "controller_server", self.controllerfeedback)
        self.random_client = self.create_client(RandomData, 'random_pose_server')
        
        # === Subscribers ===
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
        # === Publishers ===
        self.joint_pub = self.create_publisher(JointState, "/joint_command", 10)
        self.end_effector_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.singularity_pub = self.create_publisher(String, "/singularity_warning", 10)
        
        # === Timer ===
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # === TF Listener ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # === Workspace Limits ===
        self.r_max = 0.53
        self.r_min = 0.03
        
        # === State Variables ===
        self.q = np.array([0.0, -0.5, 1.0])
        self.current_q = np.array([0.0, 0.0, 0.0])
        self.current_mode = "IDLE"
        self.waiting_for_service = False
        self.wait_start_time = None
        
        # === Target Position (for continuous publishing) ===
        self.target_position = None  # [x, y, z] or None
        
        # === Teleop Variables ===
        self.cmd_vel = Twist()
        self.velocity_frame = "world"
        self.singularity_threshold = 0.005
        
        self.joint_names = ["joint_1", "joint_2", "joint_3"]
        
        self.get_logger().info("=== Controller Node Started ===")
        self.get_logger().info("Modes: IK/IPK, TO, AT/AM")

    # ==================== TRANSFORMATION MATRICES ====================
    def Rz(self, theta):
        """Rotation around Z axis"""
        c, s = cos(theta), sin(theta)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    
    def Rx(self, theta):
        """Rotation around X axis"""
        c, s = cos(theta), sin(theta)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
    
    def Trans(self, x, y, z):
        """Translation matrix"""
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

    # ==================== FORWARD KINEMATICS (from URDF) ====================
    def forward_kinematic(self, q):
        """
        FK calculated directly from URDF transforms:
        joint_1: xyz="0 0 0.2" rpy="0 0 0" axis="0 0 1"
        joint_2: xyz="0 -0.12 0" rpy="-π/2 0 0" axis="0 0 1"
        joint_3: xyz="0 -0.25 0.1" rpy="0 0 0" axis="0 0 1"
        end_effector: xyz="0 -0.28 0" rpy="π/2 0 0"
        """
        q1, q2, q3 = q[0], q[1], q[2]
        
        T_0_1 = self.Trans(0, 0, 0.2) @ self.Rz(q1)
        T_1_2 = self.Trans(0, -0.12, 0) @ self.Rx(-pi/2) @ self.Rz(q2)
        T_2_3 = self.Trans(0, -0.25, 0.1) @ self.Rz(q3)
        T_3_ee = self.Trans(0, -0.28, 0) @ self.Rx(pi/2)
        
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
        
        return T_0_ee

    def get_position(self, q):
        """Get end effector position from joint angles"""
        T = self.forward_kinematic(q)
        return T[:3, 3]

    # ==================== JACOBIAN ====================
    def compute_jacobian(self, q):
        """Compute Jacobian numerically"""
        delta = 0.0001
        J = np.zeros((3, 3))
        pos0 = self.get_position(q)
        
        for i in range(3):
            q_temp = q.copy()
            q_temp[i] += delta
            pos1 = self.get_position(q_temp)
            J[:, i] = (pos1 - pos0) / delta
        
        return J

    # ==================== INVERSE KINEMATICS ====================
    def inverse_kinematic(self, x, y, z):
        """Solve IK using optimization"""
        target = np.array([x, y, z])
        
        # Check workspace
        dist = np.linalg.norm(target - np.array([0, 0, 0.2]))
        if dist > self.r_max:
            self.get_logger().warn(f"⚠️ Out of reach! ({dist:.2f} > {self.r_max})")
            return None
        if dist < self.r_min:
            self.get_logger().warn(f"⚠️ Too close! ({dist:.2f} < {self.r_min})")
            return None
        
        # Objective function
        def objective(q):
            pos = self.get_position(q)
            return np.linalg.norm(pos - target)
        
        # Optimize
        q0 = self.q.copy()
        result = minimize(objective, q0, method='SLSQP', 
                         options={'ftol': 1e-8, 'maxiter': 200})
        
        if result.fun > 0.01:
            self.get_logger().warn(f"⚠️ IK failed! Error: {result.fun:.4f}")
            return None
        
        q_sol = result.x
        
        # Check singularity
        J = self.compute_jacobian(q_sol)
        det_J = np.linalg.det(J)
        
        if np.abs(det_J) < 0.001:
            self.get_logger().warn(f"⚠️ Singularity! det(J)={det_J:.5f}")
            return None
        
        self.get_logger().info(f"✅ IK Solution: [{q_sol[0]:.3f}, {q_sol[1]:.3f}, {q_sol[2]:.3f}], Error: {result.fun:.6f}")
        return q_sol

    # ==================== SERVICE CALLBACK ====================
    def controllerfeedback(self, request, response):
        mode = request.mode.data.upper()
        target_x = request.position.x
        target_y = request.position.y
        target_z = request.position.z
        
        if mode == "IK" or mode == "IPK":
            self.current_mode = "IK"
            self.get_logger().info(f"Mode: IK -> Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            
            q_sol = self.inverse_kinematic(target_x, target_y, target_z)
            if q_sol is not None:
                self.q = q_sol
                # Set target for continuous publishing
                self.target_position = [target_x, target_y, target_z]
                response.success = True
            else:
                self.target_position = None
                response.success = False
            return response
            
        elif mode == "TO":
            self.current_mode = "TO"
            self.target_position = None  # Clear target when in teleop
            if target_x == 1.0:
                self.velocity_frame = "end_effector"
            else:
                self.velocity_frame = "world"
            self.get_logger().info(f"Mode: TELEOPERATION (Frame: {self.velocity_frame})")
            response.success = True
            return response
        
        elif mode == "AT" or mode == "AM":
            self.current_mode = "AT"
            self.get_logger().info("Mode: AUTO -> Starting random pose loop...")
            self.waiting_for_service = False
            self.wait_start_time = None
            self.send_random_request()
            response.success = True
            return response

        else:
            self.current_mode = "IDLE"
            self.target_position = None  # Clear target
            self.get_logger().error(f"Unknown mode: {mode}")
            response.success = False
            return response

    # ==================== JOINT STATE CALLBACK ====================
    def joint_state_callback(self, msg):
        if len(msg.position) >= 3:
            self.current_q = np.array(msg.position[:3])

    # ==================== CMD_VEL CALLBACK ====================
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    # ==================== RANDOM SERVICE ====================
    def send_random_request(self):
        if not self.random_client.service_is_ready():
            self.get_logger().warn("Random Service not ready!")
            return
        
        req = RandomData.Request()
        req.mode.data = "AT"
        self.waiting_for_service = True
        future = self.random_client.call_async(req)
        future.add_done_callback(self.random_callback)

    def random_callback(self, future):
        try:
            response = future.result()
            q1 = response.position.x
            q2 = response.position.y
            q3 = response.position.z
            
            self.get_logger().info(f"Target Joints: [{q1:.2f}, {q2:.2f}, {q3:.2f}]")
            
            self.q = np.array([q1, q2, q3])
            
            # Calculate and store target position for continuous publishing
            pos = self.get_position(self.q)
            self.target_position = [pos[0], pos[1], pos[2]]
            
            self.get_logger().info(f"Moving to target: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        
        self.waiting_for_service = False

    # ==================== PUBLISH TARGET ====================
    def publish_target(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_0'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    # ==================== TELEOPERATION ====================
    def teleop_control(self):
        if self.current_mode != "TO":
            return
        
        vx = self.cmd_vel.linear.x
        vy = self.cmd_vel.linear.y
        vz = self.cmd_vel.linear.z
        
        if abs(vx) < 0.001 and abs(vy) < 0.001 and abs(vz) < 0.001:
            return
        
        # Calculate Jacobian
        J = self.compute_jacobian(self.current_q)
        det_J = np.linalg.det(J)
        
        # Check singularity
        if np.abs(det_J) < self.singularity_threshold:
            warn_msg = String()
            warn_msg.data = f"⚠️ SINGULARITY DETECTED! det(J)={det_J:.6f} - STOPPING"
            self.singularity_pub.publish(warn_msg)
            self.get_logger().warn(warn_msg.data)
            return
        
        # Velocity vector
        v_desired = np.array([vx, vy, vz])
        
        # Transform velocity based on frame
        if self.velocity_frame == "end_effector":
            T = self.forward_kinematic(self.current_q)
            R = T[:3, :3]
            v_world = R @ v_desired
        else:
            v_world = v_desired
        
        # Calculate joint velocities
        try:
            J_pinv = np.linalg.pinv(J)
            q_dot = J_pinv @ v_world
            dt = 0.02
            q_new = self.current_q + q_dot * dt
            
            # ==================== Z LIMIT PROTECTION ====================
            # Check if new position would be below ground (z < 0.05)
            new_pos = self.get_position(q_new)
            z_min = 0.05  # Minimum Z height (above link_0)
            
            if new_pos[2] < z_min:
                warn_msg = String()
                warn_msg.data = f"⚠️ Z LIMIT! z={new_pos[2]:.3f} < {z_min} - BLOCKING"
                self.singularity_pub.publish(warn_msg)
                self.get_logger().warn(warn_msg.data)
                return  # Don't update q
            
            # Check workspace limits
            dist = np.linalg.norm(new_pos - np.array([0, 0, 0.2]))
            if dist > self.r_max or dist < self.r_min:
                warn_msg = String()
                warn_msg.data = f"⚠️ WORKSPACE LIMIT! dist={dist:.3f} - BLOCKING"
                self.singularity_pub.publish(warn_msg)
                self.get_logger().warn(warn_msg.data)
                return  # Don't update q
            
            self.q = q_new
            
        except Exception as e:
            self.get_logger().error(f"Teleop error: {e}")

    # ==================== PUBLISH END EFFECTOR (from TF) ====================
    def eff_pub(self):
        try:
            t = self.tf_buffer.lookup_transform('link_0', 'end_effector', rclpy.time.Time())
            
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'link_0'
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation
            
            self.end_effector_pub.publish(msg)
        except:
            pass

    # ==================== TIMER CALLBACK ====================
    def timer_callback(self):
        self.eff_pub()
        
        # Continuously publish target if set (for RViz visualization)
        if self.target_position is not None:
            self.publish_target(self.target_position[0], 
                              self.target_position[1], 
                              self.target_position[2])
        
        if self.current_mode == "TO":
            self.teleop_control()
        
        # Publish joint command
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.q.tolist()
        self.joint_pub.publish(msg)

        # Auto mode logic
        if self.current_mode == "AT":
            error = np.linalg.norm(self.q - self.current_q)
            
            if error < 0.05 and not self.waiting_for_service:
                if self.wait_start_time is None:
                    self.wait_start_time = self.get_clock().now()
                    self.get_logger().info("Target Reached! Waiting 10 seconds...")
                else:
                    now = self.get_clock().now()
                    time_diff = (now - self.wait_start_time).nanoseconds / 1e9
                    
                    if time_diff >= 10.0:
                        self.get_logger().info("10s passed! Requesting next target...")
                        self.send_random_request()
                        self.wait_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()