#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro 
    
def generate_launch_description():
    
    pkg_name = 'control' # ชื่อ Package ต้องตรงกับโฟลเดอร์จริง
    pkg = get_package_share_directory(pkg_name)
    
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        # เชื่อม world กับ link_0 ที่ตำแหน่ง (0,0,0)
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'link_0'],
        output='screen'
    )
    
# ----------------------------------------------------
    # 2. ROBOT STATE PUBLISHER
    # ----------------------------------------------------
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    
    # [แก้ตรงนี้] เปลี่ยนจาก True เป็น False 
    # เพราะเรา run script เอง ไม่ได้ใช้ Gazebo
    parameters.append({'use_sim_time': False}) 
    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    controller_node = Node(
        package=pkg_name,
        executable='Controll_node.py', # ชื่อไฟล์ต้องตรงเป๊ะ
        output='screen'
    )

    dummy_node = Node(
        package=pkg_name,
        executable='dummy_script.py', # ชื่อไฟล์ต้องตรงเป๊ะ
        output='screen'
    )

    random_node = Node(
        package=pkg_name,
        executable='random_node.py', # เช็คชื่อไฟล์ให้ตรงกับที่ใช้ ros2 run
        name='random_pose_node',     # ตั้งชื่อ Node (Optional)
        output='screen'
    )
    
    rviz_path = os.path.join(pkg,'config','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')

    
    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(static_tf_publisher) # ต้องใส่ Node นี้
    launch_description.add_action(controller_node)
    launch_description.add_action(dummy_node)
    launch_description.add_action(random_node)
    
    return launch_description