#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro    
    
def generate_launch_description():
    
    pkg_name = 'control' # ชื่อ Package ต้องตรงกับโฟลเดอร์จริง
    pkg = get_package_share_directory(pkg_name)
    
    # ----------------------------------------------------
    # 1. STATIC TRANSFORM PUBLISHER (แก้ปัญหา Fixed Frame)
    # ----------------------------------------------------
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        # เชื่อม world กับ link_0 ที่ตำแหน่ง (0,0,0)
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'link_0'],
        output='screen'
    )
    
    # ----------------------------------------------------
    # 2. ROBOT STATE PUBLISHER (แก้ปัญหา Time Sync)
    # ----------------------------------------------------
    path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    # [CRITICAL FIX] เพิ่ม use_sim_time: True ให้ RSP ยอมรับ Timestamp
    parameters = [{'robot_description':robot_desc_xml}]
    parameters.append({'use_sim_time': True}) 
    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    # 3. CONTROLLER NODE (IK Calculator)
    controller_node = Node(
        package=pkg_name,
        executable='Controll_node.py', # ชื่อไฟล์ต้องตรงเป๊ะ
        output='screen'
    )

    # 4. DUMMY NODE (Joint State Simulator)
    dummy_node = Node(
        package=pkg_name,
        executable='dummy_script.py', # ชื่อไฟล์ต้องตรงเป๊ะ
        output='screen'
    )
    
    # 5. RVIZ
    rviz_path = os.path.join(pkg,'config','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')

    # ----------------------------------------------------
    # 6. LAUNCH ACTIONS (การจัดลำดับการรัน)
    # ----------------------------------------------------
    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(static_tf_publisher) # ต้องใส่ Node นี้
    launch_description.add_action(controller_node)
    launch_description.add_action(dummy_node)
    
    return launch_description