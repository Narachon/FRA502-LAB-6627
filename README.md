# LAB 4: 3R Kinematics Controller

ระบบควบคุมแขนกล 3-DOF พร้อม 3 โหมดการทำงาน: IK, Teleoperation, Auto

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              ROS2 System Architecture                           │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────────┐                        ┌─────────────────────────────┐ │
│  │   random_node.py    │◄──────────────────────►│     Controll_node.py        │ │
│  │                     │   /random_pose_server  │                             │ │
│  │ • Generate random   │       (Service)        │  • IK Mode (IPK)            │ │
│  │   joint angles      │                        │  • Teleop Mode (TO)         │ │
│  │ • Validate pose     │                        │  • Auto Mode (AT/AM)        │ │
│  │ • Publish /target   │                        │  • Singularity detection    │ │
│  └──────────┬──────────┘                        └──────────┬──────────────────┘ │
│             │                                              │                    │
│             │ /target                                      │ /joint_command     │
│             ▼                                              ▼                    │
│  ┌─────────────────────┐                        ┌─────────────────────────────┐ │
│  │                     │                        │     dummy_script.py         │ │
│  │       RViz2         │                        │   (Joint Simulator)         │ │
│  │                     │                        │                             │ │
│  │ • Robot Model       │                        │ • Simulate joint movement   │ │
│  │ • /target (green)   │◄───────────────────────┤ • Publish /joint_states     │ │
│  │ • /end_effector(red)│      /joint_states     │                             │ │
│  │ • TF Tree           │                        └─────────────────────────────┘ │
│  └─────────────────────┘                                                        │
│             ▲                                                                   │
│             │ /end_effector                                                     │
│             │                                                                   │
│  ┌──────────┴──────────┐      /cmd_vel          ┌─────────────────────────────┐ │
│  │  Controll_node.py   │◄──────────────────────│  Teleop_jog_keyboard.py     │ │
│  │  (TF Lookup)        │                        │                             │ │
│  └─────────────────────┘                        │ • Keyboard control          │ │
│                                                 │ • Publish velocity          │ │
│  ┌─────────────────────┐                        └─────────────────────────────┘ │
│  │ robot_state_pub     │                                                        │
│  │ (URDF → TF)         │                                                        │
│  └─────────────────────┘                                                        │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## Topics

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/target` | geometry_msgs/PoseStamped | random_node, controller | RViz | ตำแหน่งเป้าหมาย (ลูกศรสีเขียว) |
| `/end_effector` | geometry_msgs/PoseStamped | controller | RViz | ตำแหน่ง End Effector ปัจจุบัน (ลูกศรสีแดง) |
| `/joint_states` | sensor_msgs/JointState | dummy_script | robot_state_publisher | สถานะ joint ปัจจุบัน |
| `/joint_command` | sensor_msgs/JointState | controller | dummy_script | คำสั่ง joint ที่ต้องการ |
| `/cmd_vel` | geometry_msgs/Twist | teleop_keyboard | controller | ความเร็ว End Effector สำหรับ Teleop |
| `/singularity_warning` | std_msgs/String | controller | - | แจ้งเตือน Singularity |

---

## Services

| Service | Type | Server | Client | Description |
|---------|------|--------|--------|-------------|
| `/controller_server` | interface/srv/ControllerData | controller | user | เปลี่ยน Mode และส่งคำสั่ง |
| `/random_pose_server` | interface/srv/RandomData | random_node | controller | ขอตำแหน่งสุ่ม |

### Custom Service: ControllerData.srv

```
# Request
std_msgs/String mode        # "IK", "IPK", "TO", "AT", "AM"
geometry_msgs/Point position  # x, y, z

# Response  
bool success
```

### Custom Service: RandomData.srv

```
# Request
std_msgs/String mode

# Response
bool inprogress
geometry_msgs/Point position  # joint angles (q1, q2, q3)
```

---

## Part 1: Workspace

### การคำนวณ Workspace

จาก URDF ของหุ่นยนต์:
- Joint 1: Base rotation (Z-axis), height = 0.2m
- Joint 2: Shoulder, offset = -0.12m (Y)  
- Joint 3: Elbow, link = 0.25m, offset = 0.1m (Z)
- End Effector: offset = -0.28m (Y)

**Maximum Reach:** `r_max = 0.25 + 0.28 = 0.53 m`

**Minimum Reach:** `r_min = 0.03 m`

**Z Minimum:** `z_min = 0.05 m` (ไม่ให้ EE ต่ำกว่า link_0)

### วิธีตรวจสอบ Workspace

```bash
# ทดสอบ IK ที่ขอบ workspace

# ในระยะ (ควรสำเร็จ)
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'IK'}, position: {x: 0.2, y: 0.1, z: 0.3}}"

# นอกระยะ (ควร fail)
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'IK'}, position: {x: 1.0, y: 0.0, z: 0.5}}"
```

---

## Part 2: Controller Modes

### Mode 1: Inverse Kinematics (IK/IPK)

คำนวณ joint angles เพื่อไปยังตำแหน่งที่ต้องการ

```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'IK'}, position: {x: 0.2, y: 0.1, z: 0.4}}"
```

**Response:**
- `success: true` → พบคำตอบ, หุ่นยนต์เคลื่อนที่ไปตำแหน่ง
- `success: false` → ไม่พบคำตอบ (นอก workspace / singularity)

---

### Mode 2: Teleoperation (TO)

ควบคุมด้วย keyboard ผ่าน `/cmd_vel`

#### World Frame (ความเร็วเทียบกับเฟรมโลก):
```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'TO'}, position: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### End Effector Frame (ความเร็วเทียบกับเฟรม End Effector):
```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'TO'}, position: {x: 1.0, y: 0.0, z: 0.0}}"
```

#### Teleop Keyboard Controls:

| Key | Action |
|-----|--------|
| w / s | +/- X velocity |
| a / d | +/- Y velocity |
| q / e | +/- Z velocity |
| + / - | เพิ่ม/ลด ความเร็ว |
| Space | หยุด |
| Esc | ออก |

#### Singularity Warning:
เมื่อหุ่นยนต์เข้าใกล้ Singularity (det(J) < 0.005):
- หยุดการเคลื่อนที่อัตโนมัติ
- Publish ไปที่ `/singularity_warning`
- แสดงข้อความใน Terminal

```bash
# Monitor singularity warning
ros2 topic echo /singularity_warning
```

---

### Mode 3: Auto Mode (AT/AM)

สุ่มตำแหน่งและเคลื่อนที่อัตโนมัติ

```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'AT'}, position: {x: 0.0, y: 0.0, z: 0.0}}"
```

**การทำงาน:**
1. Controller ส่ง request ไปยัง `/random_pose_server`
2. Random node สุ่ม joint angles ที่ valid (ตรวจสอบ workspace, z limit, singularity)
3. Random node publish `/target` และส่ง joints กลับ
4. Controller เคลื่อนที่ไปยังตำแหน่ง
5. เมื่อถึงเป้าหมาย รอ **10 วินาที**
6. ส่ง request ใหม่ วนซ้ำไปเรื่อยๆ

---

## Installation

### 1. Clone Repository

```bash
cd ~/lab4s_ws/src
git clone <repository-url> control
```

### 2. Install Dependencies

```bash
# Python dependencies
pip3 install scipy numpy

# ROS2 dependencies (Ubuntu 22.04 + ROS2 Humble)
sudo apt install ros-humble-tf2-ros ros-humble-robot-state-publisher
```

### 3. Build

```bash
cd ~/lab4s_ws
colcon build
source install/setup.bash
```

### 4. Make Scripts Executable

```bash
chmod +x ~/lab4s_ws/src/control/scripts/*.py
```

---

## Usage

### Launch System

```bash
cd ~/lab4s_ws
source install/setup.bash
ros2 launch control simple_display.launch.py
```

### Test IK Mode

```bash
# Terminal ใหม่
source ~/lab4s_ws/install/setup.bash

# ส่งคำสั่ง IK
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'IK'}, position: {x: 0.2, y: 0.1, z: 0.4}}"
```

### Test Teleop Mode

```bash
# Terminal 1: เปลี่ยนเป็น TO mode (World Frame)
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'TO'}, position: {x: 0.0, y: 0.0, z: 0.0}}"

# Terminal 2: รัน teleop keyboard
ros2 run control Teleop_jog_keyboard.py
```

### Test Auto Mode

```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'AT'}, position: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Monitor Topics

```bash
# ดู target position
ros2 topic echo /target

# ดู end effector position
ros2 topic echo /end_effector

# ดู singularity warning
ros2 topic echo /singularity_warning
```

---

## File Structure

```
control/
├── config/
│   └── display.rviz          # RViz configuration
├── launch/
│   └── simple_display.launch.py
├── meshes/
│   ├── end_effector.stl
│   ├── link_0.stl
│   ├── link_1.stl
│   ├── link_2.stl
│   └── link_3.stl
├── robot/
│   └── visual/
│       └── my-robot.xacro    # Robot URDF
├── scripts/
│   ├── Controll_node.py      # Main controller (IK, TO, AT)
│   ├── random_node.py        # Random pose generator
│   ├── dummy_script.py       # Joint state simulator
│   └── Teleop_jog_keyboard.py # Keyboard teleop
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Troubleshooting

### 1. Service not found

```bash
# ตรวจสอบว่า nodes ทำงาน
ros2 node list

# ตรวจสอบ services
ros2 service list
```

### 2. IK Failed

- ตรวจสอบว่าตำแหน่งอยู่ใน workspace (distance < 0.53m จาก shoulder)
- ตรวจสอบว่า z > 0.05m
- ตรวจสอบว่าไม่อยู่ใกล้ singularity

### 3. Teleop ไม่ทำงาน

1. ตรวจสอบว่าอยู่ใน TO mode แล้ว
2. ตรวจสอบ `/cmd_vel` topic:
   ```bash
   ros2 topic echo /cmd_vel
   ```
3. ดู `/singularity_warning` ว่ามีการ block หรือไม่

### 4. Robot ไม่เคลื่อนที่

```bash
# ตรวจสอบ joint_command
ros2 topic echo /joint_command

# ตรวจสอบ joint_states
ros2 topic echo /joint_states
```

### 5. /target หรือ /end_effector ไม่แสดงใน RViz

1. กด **Add** → **By topic** → เลือก `/target` หรือ `/end_effector`
2. เลือก **Pose**
3. ตั้งค่า Shape เป็น **Arrow**

---

## Video Demo

[Link to demo video]

---

## Authors

- นักศึกษา FRA333

## License

MIT License
