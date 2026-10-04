# robot_kar

ROS 2 package for controlling a Doosan M1013 robot arm used for autonomous cone handling on a mobile platform.

The system consists of two main operations:

- **Cone collection (`bofafel`)** – detects a cone using a YOLO-based machine vision system, collects it from the test area, and places it onto the mobile platform according to the current platform state.
- **Cone placement (`bojarak2`)** – takes a cone from the mobile platform and places it at a fixed placement position.

The robot uses an OnRobot gripper with a custom-designed attachment for cone handling.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Doosan ROS 2 package
- Doosan M1013 robot or Doosan simulation
- OnRobot gripper for real operation
- YOLO-based machine vision system for cone collection

The Doosan ROS 2 package must be installed in the same ROS 2 workspace:

```text
https://github.com/DoosanRobotics/doosan-robot2
```

The package expects the Doosan ROS 2 interfaces and services to be available.

---

## Build

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

After changing the package source code, rebuild and source the workspace again:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Doosan ROS 2 bringup

Before running `bofafel` or `bojarak2`, the Doosan ROS 2 driver/bringup must be launched.

The bringup starts the ROS 2 interfaces used by this package, including the Doosan services under the `/dsr01` namespace. Without the Doosan bringup, the robot-arm node cannot receive and execute the commands sent by this package.

The official Doosan ROS 2 package is:

```text
https://github.com/DoosanRobotics/doosan-robot2
```

After sourcing ROS 2 and the workspace, launch the Doosan robot.

### Real M1013

For a real M1013 controller, the official Doosan ROS 2 bringup can be launched with:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=m1013
```

Adjust `host` and `port` if the robot controller uses different network settings.

The default ROS 2 robot namespace is `dsr01`, which matches the service names used by this package.

### Virtual / simulation M1013

For a virtual robot:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
```

For the Gazebo simulation:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 x:=0 y:=0
```

Keep the bringup terminal running while `bofafel` or `bojarak2` is running.

After launching the bringup, verify that the required Doosan services are available, for example:

```bash
ros2 service list | grep dsr01
```

The package can then be started from another terminal.

---

# Robot state

Before starting either operation, the Doosan robot must be in **Standby** state.

The robot does **not** have to be in the Home position.

The program waits for the robot to return to Standby after robot movements before continuing.

---

# Platform layout

The platform contains six cone columns arranged in two rows and three columns.

The front row is the row closer to the robot arm.

Viewed from the robot arm:

```text
                    Robot Arm
                       ↓

        ┌──────────┬──────────┬──────────┐
        │  RIGHT   │  CENTER  │   LEFT   │
        │          │          │          │
        │    RF    │    CF    │    LF    │  ← Front
        │          │          │          │
        │    RR    │    CR    │    LR    │  ← Rear
        │          │          │          │
        └──────────┴──────────┴──────────┘
```

The column order used by the program is:

```text
[RF, CF, LF, RR, CR, LR]
```

where:

- `RF` = Right Front
- `CF` = Center Front
- `LF` = Left Front
- `RR` = Right Rear
- `CR` = Center Rear
- `LR` = Left Rear

---

# Column heights

The `/column_heights` topic does **not** specify a placement command.

It specifies the **current number of cones already present in each column**.

Message type:

```text
std_msgs/msg/Int32MultiArray
```

The six values correspond to:

```text
[RF, CF, LF, RR, CR, LR]
```

For example:

```bash
ros2 topic pub --once /column_heights std_msgs/msg/Int32MultiArray \
"{data: [3, 2, 1, 4, 0, 2]}"
```

means:

```text
RF = 3 cones currently present
CF = 2 cones currently present
LF = 1 cone currently present
RR = 4 cones currently present
CR = 0 cones currently present
LR = 2 cones currently present
```

The `column_heights` data is used by the handling programs to determine the current occupancy of the six platform columns. The collection program uses this information to determine where the collected cone should be placed on the platform.

The configured platform has:

- 2 rows
- 3 columns
- 5 possible levels per column

---

# Cone placement

Run:

```bash
ros2 run robot_kar bojarak2
```

The placement program removes one cone from the mobile platform and places it at a **fixed placement position**.

The `/column_heights` topic is used to determine which cone positions are currently occupied on the platform, so the program knows which cone is available to be picked up.

The placement sequence is:

1. Wait for the current `/column_heights` message.
2. Determine the cone position to be removed from the platform.
3. Wait for a `/step_trigger` service call.
4. Move to the selected cone position.
5. Pick up the cone.
6. Move to the fixed placement position.
7. Release the cone.
8. Return to the home position.
9. Wait for the next step trigger.

The placement program does **not** use the machine-vision cone detection topic `/tyr/detection/cone_top_pc2`.

One `/step_trigger` call starts one placement cycle.

---

## Step trigger

`/step_trigger` is a **service**, not a topic.

Type:

```text
std_srvs/srv/Trigger
```

Call it with:

```bash
ros2 service call /step_trigger std_srvs/srv/Trigger "{}"
```

The service has no request fields.

---

# Cone collection

Run:

```bash
ros2 run robot_kar bofafel
```

The collection program uses the machine-vision system to detect the cone position.

The vision system continuously publishes the detected cone position as a 3D point.

After collecting the cone, the program uses the current `/column_heights` information to determine where the cone should be placed on the mobile platform. Therefore, `/column_heights` is used by both main operations, but for different purposes:

- `bojarak2`: determines which cone is currently present on the platform and can be removed.
- `bofafel`: determines where the newly collected cone should be placed on the platform.

## Workpiece weight verification

The collection program uses the Doosan workpiece-weight service to verify whether the cone was successfully grasped.

Service:

```text
/dsr01/force/get_workpiece_weight
```

Type:

```text
dsr_msgs2/srv/GetWorkpieceWeight
```

The program measures the weight before gripping and again after gripping.

A grasp is considered successful when:

```text
weight_after > weight_before + 0.5
```

If the measured weight increase does not reach this threshold, the collection program retries the grasp.

The configured maximum retry series is two.

This check is important when running the collection program in simulation: the simulation must provide a corresponding workpiece/gripper weight change for the grasp to be detected as successful. Simply changing the robot to simulation mode does not necessarily reproduce the physical weight change.

---

## Machine vision interface

Topic:

```text
/tyr/detection/cone_top_pc2
```

Message type:

```text
sensor_msgs/msg/PointCloud2
```

Expected coordinate frame:

```text
zed_left_camera_frame
```

The PointCloud2 message contains the cone-top position in the ZED camera coordinate system.

The program expects the point cloud to contain the following fields:

```text
x
y
z
```

The vision system continuously updates the detected position. The program uses the **first valid point** from each PointCloud2 message.

---

## Point averaging

The collection program does not use only one camera measurement for a movement.

For each position update, it collects:

```text
50 samples
```

with a sampling delay of:

```text
0.01 s
```

The 50 measurements are averaged to reduce the effect of measurement noise.

---

## Camera-to-robot coordinate transformation

The detected point is initially expressed in the ZED camera coordinate system.

The program converts it to robot coordinates using:

```text
X_robot =  Y_camera × 1000
Y_robot = -Z_camera × 1000
Z_robot = -X_camera × 1000
```

The multiplication by `1000` converts metres to millimetres.

The resulting coordinates are used for relative robot movements.

---

# Testing the machine-vision interface

The real machine-vision system can be replaced by a simulated PointCloud2 publisher.

No separate Python program is required.

The following single command publishes **one 3D point at 20 Hz**:

```bash
ros2 topic pub -r 20 /tyr/detection/cone_top_pc2 sensor_msgs/msg/PointCloud2 "{header: {frame_id: zed_left_camera_frame}, height: 1, width: 1, fields: [{name: x, offset: 0, datatype: 7, count: 1}, {name: y, offset: 4, datatype: 7, count: 1}, {name: z, offset: 8, datatype: 7, count: 1}], is_bigendian: false, point_step: 12, row_step: 12, data: [0, 0, 0, 63, 205, 204, 76, 61, 10, 215, 163, 188], is_dense: true}"
```

The command publishes:

```text
Frequency: 20 Hz
Points per message: 1
Frame: zed_left_camera_frame
```

The encoded test point is approximately:

```text
X_camera =  0.50 m
Y_camera =  0.05 m
Z_camera = -0.02 m
```

After the coordinate transformation used by `bofafel`, this corresponds approximately to:

```text
X_robot =  50 mm
Y_robot =  20 mm
Z_robot = -500 mm
```

You can verify the publisher with:

```bash
ros2 topic info /tyr/detection/cone_top_pc2
```

or:

```bash
ros2 topic echo /tyr/detection/cone_top_pc2
```

Stop the test publisher with:

```text
Ctrl+C
```

---

# Simulation

The source code contains a robot-system mode variable.

For the **real robot**, it is:

```python
robot_system_mode = 0
```

For **Doosan simulation**, change it to:

```python
robot_system_mode = 1
```

This change must be made in the relevant source file before building.

After changing the value:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

The simulation also requires the normal Doosan ROS 2 simulation services to be running.

---

# Simulation test: cone placement

The `bojarak2` program does **not** use the machine-vision system and does **not** use workpiece-weight verification.

For simulation testing, only the Doosan robot simulation and the `/column_heights` input are required.

## 1. Change to simulation mode

In `bojarak2`, change:

```python
robot_system_mode = 0
```

to:

```python
robot_system_mode = 1
```

Then rebuild:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 2. Start the Doosan simulation

Start the Doosan M1013 simulation and make sure the required Doosan ROS 2 services are available.

The robot must be in Standby state.

## 3. Publish the current platform state

For example:

```bash
ros2 topic pub --once /column_heights std_msgs/msg/Int32MultiArray \
"{data: [1, 0, 0, 0, 0, 0]}"
```

This means that the platform currently contains:

```text
RF = 1
CF = 0
LF = 0
RR = 0
CR = 0
LR = 0
```

`bojarak2` uses these values to determine which cone positions are occupied and therefore which cone should be removed from the platform.

## 4. Start the placement program

```bash
ros2 run robot_kar bojarak2
```

No `/tyr/detection/cone_top_pc2` PointCloud2 publisher is required for this test.

The program waits for a `/step_trigger` call before executing each cone-removal and placement cycle:

```bash
ros2 service call /step_trigger std_srvs/srv/Trigger "{}"
```

The program then:

1. Selects a cone position from the current `/column_heights`.
2. Moves to that position.
3. Grips the cone.
4. Moves the cone to the fixed drop position.
5. Releases the cone.
6. Returns to the home position.

## 5. Testing without the physical gripper

If the physical OnRobot gripper is not available, the gripper commands can be disabled in `bojarak2`.

The following lines are the gripper-related parts of the supplied code and can be commented out:

### Gripper import

```python
from robot_kar.onrobot import RG
```

### Gripper initialization

```python
self.gripper = RG(gripper, ip, port)
```

### Opening before the pick

```python
self.gripper.open_gripper()
```

### Closing after reaching the cone

```python
self.gripper.close_gripper()
```

### Opening after reaching the fixed drop position

```python
self.gripper.open_gripper()
```

With these lines commented out, the robot still executes the programmed pick and drop movements, but no physical gripper commands are sent.

No workpiece-weight lines need to be disabled in `bojarak2`, because this program does not use the workpiece-weight service.

# Simulation test: cone collection

The `bofafel` program uses the machine-vision interface to locate the cone. Unlike `bojarak2`, it also uses the Doosan workpiece-weight measurement to verify whether the cone was successfully grasped.

## 1. Change to simulation mode

In `bofafel`, change:

```python
robot_system_mode = 0
```

to:

```python
robot_system_mode = 1
```

Then rebuild:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 2. Start the Doosan simulation

Start the Doosan M1013 simulation.

The robot must be in Standby state.

## 3. Start the simulated vision publisher

For testing the machine-vision interface without the physical ZED system, publish one PointCloud2 point at 20 Hz:

```bash
ros2 topic pub -r 20 /tyr/detection/cone_top_pc2 sensor_msgs/msg/PointCloud2 "{header: {frame_id: zed_left_camera_frame}, height: 1, width: 1, fields: [{name: x, offset: 0, datatype: 7, count: 1}, {name: y, offset: 4, datatype: 7, count: 1}, {name: z, offset: 8, datatype: 7, count: 1}], is_bigendian: false, point_step: 12, row_step: 12, data: [0, 0, 0, 63, 205, 204, 76, 61, 10, 215, 163, 188], is_dense: true}"
```

This replaces the real machine-vision input with a fixed cone position.

## 4. Start the collection program

```bash
ros2 run robot_kar bofafel
```

The program uses the PointCloud2 data to calculate the cone position and performs the collection sequence.

The `/column_heights` topic is also required because the program uses the current platform state to determine where the collected cone should be placed.

For example:

```bash
ros2 topic pub --once /column_heights std_msgs/msg/Int32MultiArray \
"{data: [1, 0, 0, 0, 0, 0]}"
```

## 5. Workpiece-weight verification

The `bofafel` code checks the workpiece weight before and after the grasp.

The relevant code is:

```python
weight = self.get_workpiece_weight()
```

and after gripping:

```python
weight2 = self.get_workpiece_weight()
```

The grasp is accepted when:

```python
if weight2 > weight + 0.5:
    succes = True
```

If the physical workpiece-weight measurement is not available in the simulation, these parts should be disabled.

### Workpiece-weight service import

Comment out:

```python
from dsr_msgs2.srv import DrlStart, GetRobotState, GetWorkpieceWeight
```

and use:

```python
from dsr_msgs2.srv import DrlStart, GetRobotState
```

### Workpiece-weight client

Comment out:

```python
self.workpiece_cli = self.create_client(GetWorkpieceWeight, '/dsr01/force/get_workpiece_weight')
```

### Waiting for the service

Comment out the complete block:

```python
while not self.workpiece_cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Waiting for workpiece_weight...')
```

### Workpiece-weight function

Comment out the complete function:

```python
def get_workpiece_weight(self):
    request = GetWorkpieceWeight.Request()
    future = self.workpiece_cli.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    if future.result() is None:
        return None
    return future.result().weight
```

### Weight measurement before grasping

Comment out:

```python
weight = self.get_workpiece_weight()
```

### Weight measurement after grasping and verification

Comment out:

```python
weight2 = self.get_workpiece_weight()
if weight2 > weight + 0.5:
    succes = True
    retry -= 1
elif retry == max_series:
    break
else:
    retry += 1
    continue
```
After that section add a plus line to prevent an infinite cycle:

```python
succes = True
```

If this verification block is disabled, the retry loop must also be adjusted so that the program does not wait for a condition that has been removed. For a simulation where grasp verification is not required, the successful-grasp state should be set directly after the grasp and subsequent movement.

## 6. Testing without the physical gripper

The physical OnRobot gripper can also be disabled independently.

Comment out the import:

```python
from .onrobot import RG
```

Comment out the initialization:

```python
self.rg = RG(args.gripper, args.ip, args.port)
```

Comment out the three gripper commands:

```python
self.rg.open_gripper()
```

```python
self.rg.close_gripper()
```

```python
self.rg.open_gripper()
```

These correspond respectively to:

1. opening before approaching the cone,
2. closing to grasp the cone,
3. opening at the end of the placement sequence.

If both the physical gripper and workpiece-weight verification are unavailable, both sets of lines described above should be disabled.

# Main ROS 2 interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/column_heights` | `std_msgs/msg/Int32MultiArray` | Current number of cones in the six platform columns |
| `/step_trigger` | `std_srvs/srv/Trigger` | Starts one handling/placement step |
| `/tyr/detection/cone_top_pc2` | `sensor_msgs/msg/PointCloud2` | Continuously updated detected cone position |
| `/dsr01/drl/drl_start` | `dsr_msgs2/srv/DrlStart` | Sends DRL code to the Doosan controller |
| `/dsr01/system/get_robot_state` | `dsr_msgs2/srv/GetRobotState` | Reads the Doosan robot state |
| `/dsr01/force/get_workpiece_weight` | `dsr_msgs2/srv/GetWorkpieceWeight` | Measures workpiece weight for grasp verification |

---

# Executables

## Cone collection

```bash
ros2 run robot_kar bofafel
```

Uses the PointCloud2-based machine-vision interface to locate and collect cones from the platform.

## Cone placement

```bash
ros2 run robot_kar bojarak2
```

Uses the current `/column_heights` values to determine which cone is present on the platform and removes that cone, then places it at a fixed placement position.

---

# Typical startup sequence

## Real robot

Source ROS 2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
```

First launch the Doosan ROS 2 bringup in its own terminal:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=m1013
```

Keep this terminal running.

Then, in another terminal, start cone collection:

```bash
ros2 run robot_kar bofafel
```

or start cone placement:

```bash
ros2 run robot_kar bojarak2
```

The real robot must be in Standby state.

The real robot uses:

```python
robot_system_mode = 0
```

---

## Simulation

Change:

```python
robot_system_mode = 0
```

to:

```python
robot_system_mode = 1
```

Then rebuild:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Start the Doosan simulation/bringup in its own terminal. For Gazebo:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 x:=0 y:=0
```

Keep the bringup/simulation terminal running.

Then start `bofafel` or `bojarak2` from another terminal.

For vision testing, replace the real ZED input with the 20 Hz PointCloud2 command described above.

---

# Quick command reference

### Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Start cone collection

```bash
ros2 run robot_kar bofafel
```

### Start cone placement

```bash
ros2 run robot_kar bojarak2
```

### Set current platform heights

```bash
ros2 topic pub --once /column_heights std_msgs/msg/Int32MultiArray \
"{data: [1, 0, 0, 0, 0, 0]}"
```

### Trigger one step

```bash
ros2 service call /step_trigger std_srvs/srv/Trigger "{}"
```

### Check machine-vision topic

```bash
ros2 topic info /tyr/detection/cone_top_pc2
```

### Display machine-vision data

```bash
ros2 topic echo /tyr/detection/cone_top_pc2
```

### Simulated PointCloud2 at 20 Hz

```bash
ros2 topic pub -r 20 /tyr/detection/cone_top_pc2 sensor_msgs/msg/PointCloud2 "{header: {frame_id: zed_left_camera_frame}, height: 1, width: 1, fields: [{name: x, offset: 0, datatype: 7, count: 1}, {name: y, offset: 4, datatype: 7, count: 1}, {name: z, offset: 8, datatype: 7, count: 1}], is_bigendian: false, point_step: 12, row_step: 12, data: [0, 0, 0, 63, 205, 204, 76, 61, 10, 215, 163, 188], is_dense: true}"
```

---

# Notes

- `/column_heights` describes the **current platform state**, not the desired placement positions.
- `bojarak2` uses `/column_heights` to determine which cone to remove from the platform.
- `bofafel` uses `/column_heights` to determine where the newly collected cone should be placed on the platform.
- `/step_trigger` is a ROS 2 service, not a topic.
- `/tyr/detection/cone_top_pc2` is a `sensor_msgs/msg/PointCloud2` topic.
- The vision system continuously updates one cone position at a time.
- `bofafel` averages 50 valid point samples before using the detected position.
- Camera coordinates are converted from metres to robot millimetres using the transformation documented above.
- Real operation uses `robot_system_mode = 0`.
- Simulation uses `robot_system_mode = 1`.
- A full simulation of grasp verification requires the workpiece-weight response to behave appropriately.
- The robot must be in Standby state before operation.
