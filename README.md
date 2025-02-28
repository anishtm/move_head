# move_head: ROS Package for Humanoid Head Movement

`move_head` is a ROS package built on ROS Noetic. It allows control of a humanoid head with movement along two axes: **pan** (X-axis) using a Nema 17 stepper motor and **tilt** (Y-axis) using a 25kg servo motor. The system also includes smaller servos for eye movement and a jaw movement servo.

---

## Setup Instructions

### 1. Create ROS Workspace

To create the `move_head` package, follow the normal ROS workspace creation procedure:

```bash
cd ~/catkin_ws/src
git clone https://github.com/anishtm/move_head.git
cd ..
catkin_make clean
catkin_make
```

ROS Noetic is used despite nearing its LTS end due to the inclusion of the **rosserial** package, which is required for communication with Arduino. Although two Arduino boards (Uno and Mega) are used for logistical and PCB issues, a single Arduino Mega could suffice for this operation.

---

## Homing Sequence for Head Movement

### Servo Homing:
- **Default upright position**: 84 degrees
- **Upper max angle**: 64 degrees
- **Lower max angle**: 100 degrees

### Stepper Homing:
- There are two limit switches at each end of the stepper's range.
  - **Left switch**: Assigned as position 0.
  - **Right switch**: Normal position at approximately 4670 ± 10 steps (as indicated by `AccelStepper.currentPosition()`).

### Homing Process:
1. The system starts at the default upright position of 84°.
2. The head is moved left until the left limit switch is triggered.
3. The head is then moved slightly right, and once the right limit switch is triggered, the position is set to 0.
4. This process is performed using an **Arduino Uno** and publishes `geometry_msgs/Point32` on the `/commands` topic for communication. The `x` value represents the stepper position, the `y` value represents the angle, and the `z` value indicates the action type.

---

## Action Types

The following actions are supported by the system:

- **0**: Calibration
- **1**: Move to center
- **2**: Move to angle
- **3**: Move to position (Stepper)
- **4**: Move to both angle and position simultaneously
- **5**: Move to both angle and position, one at a time

---

## Arduino Sketches

There are three main Arduino sketches used in this system, each subscribing to the `/commands` topic:

- **`message_checker.ino`**: Verifies that messages are being received correctly.
- **`ros_controller_almost.ino`**: Publishes commands and manages motor control.
- **`ros_controller_final.ino`**: Controls motor movement without publishing any commands.

Additionally, there is a dedicated sketch for jaw movement:

- **`jaw_controller.ino`**: Subscribes to both `/voice` and `/commands`. The jaw only moves when a valid command is received on `/commands`.

For eye movement, the servos will only move when a **centering command** is received through the `/commands` topic.

---

## Python Nodes

The package also includes Python nodes for face detection and tracking using **Yunet** and the **SORT** tracking algorithm:

- **`live_feed.py`**: Captures images from the webcam.
- **`detector.py`**: Detects faces, sends movement commands, and visualizes the process.
- **`detector_without_visualization.py`**: Similar to `detector.py`, but without the visualization.

There is also a debugging node that publishes manual commands to the Arduino.

---

## Voice Topic

The **voice topic** is managed by another node, which is beyond the scope of this README.

---

## Starting the System

1. Start the ROS master:
   ```bash
   roscore
   ```

2. Run the live feed node:
   ```bash
   rosrun move_head live_feed.py
   ```

3. Run the face detection and tracking node (with or without visualization):
   ```bash
   rosrun move_head detector.py
   # or
   rosrun move_head detector_without_visualization.py
   ```

4. Launch the Arduino serial nodes for head and jaw movement:
   ```bash
   rosrun rosserial_arduino serial_node.py __name:=head /dev/ttyACM0 _baud:=115200
   rosrun rosserial_arduino serial_node.py __name:=jaw /dev/ttyACM1 _baud:=115200
   ```

   **Note:** Make sure the **head movement Arduino** is connected to `/dev/ttyACM0` and the **jaw movement Arduino** is connected to `/dev/ttyACM1`.

---

## Calibration

If the calibration doesn't start automatically, you can manually trigger it with the following command:

```bash
rostopic pub /commands geometry_msgs/Point32 "x: 0 y: 0 z: 0"
```

---

## Optional: Run the AI Node

You can also run the optional AI node if available:

```bash
rosrun kaanchii_ai kaanchii_ai.py
```

---

## Disclaimer

This README is intended to serve as a reminder for setting up and working with this project. The setup process and code are specific to the current version and may require adjustments for future updates.

---
