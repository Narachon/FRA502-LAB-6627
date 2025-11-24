#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class Jointstate(Node):
    def __init__(self):
        super().__init__('Joint')
        
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            JointState,
            "joint_command", 
            self.command_callback,
            10
        )
        
        self.dt = 0.02
        self.create_timer(self.dt, self.sim_loop)
        
        # ✅ FIX: กำหนดทั้งสองตัวเป็น NumPy Array ตั้งแต่ต้น
        self.q = np.array([0.0, 0.0, 0.0])
        self.target_q = np.array([0.0, 0.0, 0.0]) # ต้องเป็น np.array ด้วย
        
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.get_logger().info(">>> DUMMY NODE STARTED SUCCESSFULLY! <<<")

    def command_callback(self, msg):
        if len(msg.position) >= 3:
            # ✅ FIX: แปลง input เป็น NumPy Array ทันทีที่รับเข้ามา
            self.target_q = np.array(msg.position) 

    def sim_loop(self):
        # [จุดเช็ค] ต้องขึ้นรัวๆ
        self.get_logger().info(f"Publishing joints: {self.q}") 
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        speed = 2.0 * self.dt 
        
        # 🟢 Logic: ขยับเข้าหาเป้าหมาย (ทั้งหมดเป็น NumPy Operations)
        diff = self.target_q - self.q 
        
        for i in range(3):
            if np.abs(diff[i]) > speed:
                self.q[i] += speed * np.sign(diff[i])
            else:
                self.q[i] = self.target_q[i]

        # ✅ FIX: แปลงกลับเป็น list เพื่อ publish
        msg.position = self.q.tolist() 
        msg.name = self.name
        self.joint_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = Jointstate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()