#!/usr/bin/python3

import rclpy
from rclpy.node import Node
# เช็คชื่อ package ให้ตรงกับโฟลเดอร์จริง (interface หรือ robot_interface)
from interface.srv import ControllerData 
import roboticstoolbox as rtb
import numpy as np
from math import pi
from spatialmath import SE3

# [แก้จุดที่ 1] ต้อง Import Message JointState ไม่งั้น Error
from sensor_msgs.msg import JointState 

class Controller(Node):
    def __init__(self):
        super().__init__('Control')
        self.srv = self.create_service(ControllerData, "controller_server", self.controllerfeedback)
        self.joint_pub = self.create_publisher(JointState, "joint_command", 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.r_max = 0.28 + 0.25
        self.r_min = 0.03
        self.l = 0.2
        self.inverse_setpoint = [0.0, 0.0, 0.0]
        self.q = np.array([0.1, 0.3, 0.2])
        self.current_mode = "IDLE"
        
        # [แก้จุดที่ 2] ต้องกำหนดชื่อ Joint ให้ตรงกับไฟล์ URDF ของหุ่นยนต์
        # (เช่น joint_1, joint_2, joint_3 หรือ link1_to_link2 แล้วแต่ที่คุณตั้งใน URDF)
        self.joint_names = ["joint_1", "joint_2", "joint_3"] 
        
        L1 = rtb.RevoluteMDH(d=0.2, a=0.0, alpha=0.0)
        L2 = rtb.RevoluteMDH(d=0.02, a=0.0, alpha=np.pi/2)
        L3 = rtb.RevoluteMDH(d=0.0, a=0.25, alpha=0.0)
        
        self.robot = rtb.DHRobot(
            [L1, L2, L3],
            tool=SE3.Tx(0.28), 
            name="My_3DOF_Arm"
        )
        self.get_logger().info("Controller Started")
        
    def controllerfeedback(self, request , response):
        mode = request.mode.data # ดึงค่า Mode
        
        target_x, target_y, target_z = request.position.x, request.position.y, request.position.z
        q_sol = None
        
        # ------------------------------------------------------------------
        # 1. Inverse Kinematics Mode (IPK)
        # ------------------------------------------------------------------
        if mode == "IK":
            self.current_mode = "IK"
            self.inverse_setpoint = [target_x, target_y, target_z]
            q_sol = self.inverse_kinematic(target_x, target_y, target_z) # คำนวณ IK ทันที [cite: 10]
            
            # การเคลื่อนที่ (self.q = q_sol) จะเกิดขึ้นในส่วน Check Result ด้านล่าง
        
        # ------------------------------------------------------------------
        # 2. Teleoperation Mode (TO)
        # ------------------------------------------------------------------
        elif mode == "TO":
            self.current_mode = "TO"
            self.get_logger().info("Mode set to TELEOPERATION (TO). Waiting for /cmd_vel.")
            # *Mode นี้ Controller จะต้องรอรับ Topic /cmd_vel (Twist)
            # [cite_start]*แล้วใช้ Jacobian เพื่อคำนวณ q_dot (ซึ่ง Logic อยู่ใน timer_callback) 
            response.success = True
            return response # ออกจากฟังก์ชันทันที
        
        # ------------------------------------------------------------------
        # 3. Auto Mode (AM)
        # ------------------------------------------------------------------
        elif mode == "AM":
            self.current_mode = "AM"
            self.get_logger().info("Mode set to AUTO (AM). Requesting first random pose...")
            
            # [cite_start]*ต้องส่ง Service request ไปหา Random Node เพื่อขอตำแหน่งเริ่มต้น 
            # self.send_random_pose_request() 
            
            response.success = True
            return response # ออกจากฟังก์ชันทันที
        
        else:
            self.get_logger().error(f"Unknown mode requested: {mode}")
            response.success = False
            return response

        # ------------------------------------------------------------------
        # 4. Check IK Result (ทำงานเฉพาะเมื่อ mode == "IK" เท่านั้น)
        # ------------------------------------------------------------------
        if q_sol is not None:
            self.get_logger().info(f"IK Success: {q_sol}")
            self.q = q_sol     # หุ่นยนต์เคลื่อนที่ไปยัง Solution ที่คำนวณได้ [cite: 11]
            response.success = True
        else:
            self.get_logger().error("IK Failed. Arm remains stationary.")
            response.success = False # return False หากไม่พบคำตอบ [cite: 11]
        
        return response
    
    def inverse_kinematic(self, x, y, z):
        target_vec = np.array([x, y, z - 0.2]) 
        dist = np.linalg.norm(target_vec)

        if not (self.r_min <= dist <= self.r_max):
            self.get_logger().error(f"Target out of reach! (Distance: {dist:.3f})")
            return None

        target_pose = SE3(x, y, z)
        
        sol = self.robot.ikine_LM(
            target_pose,
            mask=[1, 1, 1, 0, 0, 0], 
            q0=self.q,               
            joint_limits=False,
            ilimit=100,              
            tol=1e-6                 
        )

        if sol.success:
            return sol.q
        
        self.get_logger().error("IK Solver failed to converge.")
        return None
    
    def timer_callback(self):
        msg = JointState()
        
        # [แก้จุดที่ 3] ใส่เวลาปัจจุบัน (Timestamp)
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # [แก้จุดที่ 2 ต่อ] ใส่ชื่อ Joint ลงไปใน message
        msg.name = self.joint_names
        
        msg.position = self.q.tolist() 
        self.joint_pub.publish(msg)             

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()