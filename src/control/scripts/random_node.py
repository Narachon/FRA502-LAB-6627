#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import random
from math import cos, sin, pi
from interface.srv import ControllerData # <--- [IMPORT ที่เพิ่ม]

# ... (ค่าคงที่ R_MIN, R_MAX, Z_OFFSET, Z_MAX_REACH, Z_MIN_REACH เหมือนเดิม) ...

class RandomPosePublisher(Node):
    def __init__(self):
        super().__init__('random_pose_node')
        
        # 1. Publisher สำหรับ Topic /target (Part 1 Q2)
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        
        # 2. Timer สำหรับ Publish อัตโนมัติ (Part 1 Q2)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # 🟢 ADD: 3. Service Server สำหรับ Auto Mode (AM) (Part 2 Q3)
        self.srv = self.create_service(
            ControllerData, 
            "random_pose_server", 
            self.random_service_callback
        )
        
        self.get_logger().info("Random Pose Publisher Node Started.")
        
    # ... (def generate_random_pose เหมือนเดิม) ...

    # 🟢 ADD: Callback Function ที่ Controller จะเรียกใช้
    def random_service_callback(self, request, response):
        """รับ Service Request (AM) และส่งตำแหน่งสุ่มกลับไป"""
        self.get_logger().info('Received AM request. Generating new pose.')
        
        # 1. สุ่มตำแหน่งใหม่
        x, y, z = self.generate_random_pose()
        
        # 2. ตอบกลับด้วยพิกัดที่สุ่มได้ (ใช้ structure จาก ControllerData.srv)
        response.success = True
        response.position.x = x
        response.position.y = y
        response.position.z = z
        
        # 3. Publish ตำแหน่งนี้ออกไปที่ /target ด้วย (เพื่อแสดงผลใน Rviz)
        self.publish_pose_stamped(x, y, z) 
        
        return response

    # 🟢 Helper function: เพื่อใช้ publish ซ้ำ
    def publish_pose_stamped(self, x, y, z):
        """สร้างและ Publish PoseStamped (แยก logic ออกมา)"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_0' 
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0 
        self.target_pub.publish(msg)
        self.get_logger().info(f"Published random target: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

    def timer_callback(self):
        """Timer callback (ยังคง Publish ต่อเนื่องตาม Part 1 Q2)"""
        # Note: คุณอาจจะปิด Timer ไปเลยก็ได้ หากต้องการให้ Random Node ทำงานแค่ตอน Controller สั่ง
        x, y, z = self.generate_random_pose()
        self.publish_pose_stamped(x, y, z) 
        
def main(args=None):
    rclpy.init(args=args)
    node = RandomPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()