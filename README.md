This a package to move robotic head of humanoid robot  "Kanchii" developed by Robotics Association Nepal (RAN).

This package contains 4

1. Arduino Head Controller - Controls stepper and servo for pan and tilt using serial communication
2. Arduino People Jaw - Receives data from people sensor about human detection and controls jaw for mouth movement using ROS Noetic.
3. Arduino Commander - Python node used to give commands to arduino connected through serial communication
4. Controller - It gives actual commands on what to do how to control the motors. It publishes topic for arduino_commander which then commands to arduino

To control the head
1. Flash arduino with Arduino head controller
2. Flash arduino mega with arduino people jaw - uno doesnt work de to memory issues, esp32 doesnt work due to servo.h issues
3. connect both to the host ros machines
4. find the arduino physical location usually /dev/ttyACM0 and /dev/ttyACM1 for arduino and /dev/ttyUSB0 for esp32
5. start ros: roscore
6. start arduino commander : rosrun move_head arduin_commander.py _serial_port:= "/dev/ttyACM1" if its ACM0 it is by default the
7. start arduino people jaw: rosrun rosserial_arduino serial_node.py /dev/ttyACM0
8. start controller: rosrun move_head controller.py


now if you want to use conversation capabilties, you need to get kaanchii_ai package developed by Er. Tejash Katwal at RAN


note this work is incomplete. It is just for the sake so that it makes when i read it after few months
