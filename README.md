# 3R Robot Control & Visualization (ROS 2)

This project implements a control system for a 3-DOF (Revolute-Revolute-Revolute) Robot Arm using ROS 2. It features Jacobian-based teleoperation, Inverse Kinematics (IK) trajectory planning, workspace validation, and an automated random pose scheduler.

## Dependencies

Ensure you have the following installed in your ROS 2 environment:

* ROS 2 Humble
* SciPy: `pip3 install scipy`
* NumPy: `pip3 install numpy`

**Note:** This package requires a custom interface package named `interface` containing the following services:

* `ControllerData.srv` (Request: `std_msgs/String mode`, `geometry_msgs/Point position` | Response: `bool success`)
* `RandomData.srv` (Request: `std_msgs/String mode` | Response: `geometry_msgs/Point position`, `bool inprogress`)

---

## File Descriptions

### 1. `Controll_node.py` (Main Controller)

The core node that handles the robot's kinematics and control logic.

* **Kinematics:** Uses URDF-based transformation matrices for FK/IK calculations.
* **Modes:**
  * `IK/IPK`: Inverse Kinematics - moves robot to target xyz position using scipy optimization.
  * `TO`: Teleoperation - velocity control using Jacobian Matrix (J⁻¹·v). Includes Singularity Avoidance.
  * `AT/AM`: Auto Mode - requests random poses and moves automatically.
* **Outputs:** Publishes `/joint_command`, `/end_effector`, `/target`, `/singularity_warning`.

### 2. `Teleop_jog_keyboard.py` (Keyboard Input)

A keyboard interface for manual velocity control.

* **Output:** `/cmd_vel` (Twist message).
* **Features:** Adjustable speed, intuitive WASD+QE controls.

### 3. `random_node.py` (Target Generator)

Generates valid random joint configurations for the robot.

* **Validation:** Checks if the random pose is:
  * Within workspace (r_max = 0.53m, r_min = 0.03m)
  * Above ground level (z > 0.05m)
  * Not in singularity (det(J) > 0.005)
* **Service:** Provides the `/random_pose_server` service.
* **Output:** Publishes `/target` for RViz visualization.

### 4. `dummy_script.py` (Joint Simulator)

Simulates joint movement by interpolating toward target positions.

* **Input:** `/joint_command` from controller.
* **Output:** `/joint_states` for robot_state_publisher and TF.

---

## Installation

### Install dependencies

```bash
pip3 install scipy numpy
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-tf2-ros
```

### Clone and build

```bash
cd ~/lab4s_ws/src
git clone <repository-url>
cd ~/lab4s_ws
colcon build
source install/setup.bash
```

### Make scripts executable

```bash
chmod +x ~/lab4s_ws/src/control/scripts/*.py
```

---

## How to Run

Open 3 Terminals and run in order:

### Terminal 1 - Launch System

```bash
source ~/lab4s_ws/install/setup.bash
ros2 launch control simple_display.launch.py
```

### Terminal 2 - Service Calls (Mode Control)

```bash
source ~/lab4s_ws/install/setup.bash
# See Service API below
```

### Terminal 3 - Teleop (Optional)

```bash
source ~/lab4s_ws/install/setup.bash
ros2 run control Teleop_jog_keyboard.py
```

---

## Teleop Controls

When running `Teleop_jog_keyboard.py`, use the following keys:

| Key | Action |
|-----|--------|
| `w` | Move +X (Forward) |
| `s` | Move -X (Backward) |
| `a` | Move +Y (Left) |
| `d` | Move -Y (Right) |
| `q` | Move +Z (Up) |
| `e` | Move -Z (Down) |
| `+` | Increase Speed |
| `-` | Decrease Speed |
| `Space` | Stop Movement |
| `Esc` | Quit |

---

## Service API

You can control the system modes using ROS 2 service calls:

### 1. Inverse Kinematics Mode (IK)

Move robot to specific xyz position:

```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'IK'}, position: {x: 0.2, y: 0.1, z: 0.4}}"
```

### 2. Teleoperation Mode - World Frame

```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'TO'}, position: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3. Teleoperation Mode - End Effector Frame

```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'TO'}, position: {x: 1.0, y: 0.0, z: 0.0}}"
```

### 4. Auto Mode (Random Pose Loop)

```bash
ros2 service call /controller_server interface/srv/ControllerData "{mode: {data: 'AT'}, position: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Technical Details

### Workspace

* **Maximum Reach:** r_max = 0.53m (from shoulder joint at z=0.2)
* **Minimum Reach:** r_min = 0.03m
* **Z Limit:** z_min = 0.05m (prevents collision with base)

### Forward Kinematics (from URDF)

```python
T_0_1 = Trans(0, 0, 0.2) @ Rz(q1)
T_1_2 = Trans(0, -0.12, 0) @ Rx(-π/2) @ Rz(q2)
T_2_3 = Trans(0, -0.25, 0.1) @ Rz(q3)
T_3_ee = Trans(0, -0.28, 0) @ Rx(π/2)
T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
```

### Singularity Handling

The controller checks `det(J)` continuously:
* If `abs(det_J) < 0.005`, the robot stops immediately
* Warning message published to `/singularity_warning`
* Displayed in terminal

### Frame Handling

In Teleoperation mode, `position.x` in service request determines frame:
* `0.0` → World Frame
* `1.0` → End Effector Frame

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/target` | PoseStamped | Target position (green arrow in RViz) |
| `/end_effector` | PoseStamped | Current EE position (red arrow in RViz) |
| `/joint_states` | JointState | Current joint angles |
| `/joint_command` | JointState | Commanded joint angles |
| `/cmd_vel` | Twist | Velocity command for teleop |
| `/singularity_warning` | String | Singularity alert message |

---

## RViz Visualization

To display `/target` and `/end_effector` arrows in RViz:

1. Click **Add** → **By topic**
2. Select `/target` → **Pose** → OK
3. Select `/end_effector` → **Pose** → OK
4. Set Shape to **Arrow** and choose colors (Green for target, Red for EE)

---

## Authors

FRA333 - Robotics Studio

## License

MIT License
