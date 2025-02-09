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


#!/bin/bash

# Function to check if rosserial_arduino connects successfully
check_serial_connection() {
    local port=$1
    echo "Checking connection on $port..."
    rosrun rosserial_arduino serial_node.py "$port" &> /tmp/rosserial_output.log &
    local pid=$!
    sleep 5  # Give it time to connect
    kill $pid  # Kill the process after checking

    if grep -q "Unable to sync with device" /tmp/rosserial_output.log; then
        echo "Failed to connect on $port"
        return 1
    elif grep -q "Setup publisher on" /tmp/rosserial_output.log; then
        echo "Successfully connected on $port"
        return 0
    else
        echo "Unknown response on $port"
        return 1
    fi
}

# Detect available ports
PORT1="/dev/ttyACM0"
PORT2="/dev/ttyACM1"
SELECTED_PORT=""

# Try the first port
check_serial_connection "$PORT1"
if [ $? -eq 0 ]; then
    SELECTED_PORT="$PORT1"
else
    check_serial_connection "$PORT2"
    if [ $? -eq 0 ]; then
        SELECTED_PORT="$PORT2"
    else
        echo "No valid serial connection found. Exiting..."
        exit 1
    fi
fi

# Start roscore
echo "Starting roscore..."
roscore & 
sleep 5  # Wait for roscore to initialize

# Start move_head Arduino commander
echo "Starting move_head arduin_commander.py on $SELECTED_PORT..."
rosrun move_head arduin_commander.py _serial_port:="$SELECTED_PORT" &
sleep 3  # Allow some time to initialize

# Start rosserial_arduino
echo "Starting rosserial_arduino on $SELECTED_PORT..."
rosrun rosserial_arduino serial_node.py "$SELECTED_PORT" &
sleep 3

# Start move_head controller
echo "Starting move_head controller.py..."
rosrun move_head controller.py &
sleep 3

# Start kaanchii_ai
echo "Starting kaanchii_ai..."
rosrun kaanchii_ai kaanchii_ai.py &

echo "All processes started successfully!"


i want to create a file that 1. starts a roscore

2. then rosrun move_head arduin_commander.py _serial_port:= "/dev/ttyACM0"

3. then runs rosrun rosserial_arduino serial_node.py /dev/ttyACM1 

4. then rosrun move_head controller.py

5. then rosrun kaanchii_ai kaanchii_ai.py 

it may be bash file
it may be python file

but it has to run on start up on ubuntu 20 and it should be able to see list of port address like /dev/ttyACM0 and ACM1, 

first it should run 3. if it is correct it should show this
vboxuser@Ubuntu:~$ rosrun rosserial_arduino serial_node.py /dev/ttyACM0
[INFO] [1739086646.983785]: ROS Serial Python Node
[INFO] [1739086646.992316]: Connecting to /dev/ttyACM0 at 57600 baud
[INFO] [1739086649.114027]: Requesting topics...
[INFO] [1739086649.157318]: Note: publish buffer size is 512 bytes
[INFO] [1739086649.159285]: Setup publisher on face_detection [move_head/Face]
[INFO] [1739086649.164887]: Note: subscribe buffer size is 512 bytes
[INFO] [1739086649.166376]: Setup subscriber on /voice [std_msgs/String]
[INFO] [1739086649.422126]: No faces detected
[INFO] [1739086649.920068]: No faces detected
[INFO] [1739086650.421165]: No faces detected

if is wrong it should show this
vboxuser@Ubuntu:~$ rosrun rosserial_arduino serial_node.py /dev/ttyACM0
[INFO] [1739086670.642302]: ROS Serial Python Node
[INFO] [1739086670.648244]: Connecting to /dev/ttyACM0 at 57600 baud
[INFO] [1739086672.761604]: Requesting topics...
[ERROR] [1739086687.764509]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino

then choose the other one


and the host machine has username and password, can i set it up so that, on start up it enters its username and password, starts up the file ??
