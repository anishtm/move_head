#include <ros.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/String.h>
#include "head_movement.h"
#include "jaw_movement.h"

ros::NodeHandle nh;
geometry_msgs::Point32 latest_msg;

void messageCb(const geometry_msgs::Point32& msg) {
    latest_msg = msg;
}

void voiceCallback(const std_msgs::String& msg) {
    //latest_msg = msg;
    processJawMovement(msg.data);
}
 
ros::Subscriber<geometry_msgs::Point32> command_sub("/commands", messageCb);
ros::Subscriber<std_msgs::String> voice_sub("/voice", &voiceCallback);



void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(command_sub);


    nh.subscribe(voice_sub);

    setupHeadMovement();

    setupJawMovement(nh);

    latest_msg.x = -1;

    nh.loginfo("Setup complete, waiting for messages...");
}

void loop() {
    nh.spinOnce();
    if (latest_msg.x != -1){
      process_msg(latest_msg);
    }
    updateJawMovement();
    delay(1);
}

void process_msg(const geometry_msgs::Point32& msg){
    uint8_t type = int(msg.z);
    int16_t position = int(msg.x);
    uint8_t angle = int(msg.y);

    switch (type) {
        case 0:  
            calibration();
           // chatter.publish(&str_msg);
            break;
        case 1:  
            // center
            syncMovement(CENTER, UPRIGHT_POSITION);
            break;
        case 2:  
            // servo
            sinusoidalSmoothing(current_angle, angle);
            break;
        case 3:  
            // stepper
            syncMovement(position, angle);
            break;
        case 4:  
            // both & together
            syncMovement(position, angle);
            break;
        case 5:  
            // both but one by one
            sinusoidalSmoothing(current_angle, angle);
            move_to_position(position);
            break;
        default:
            nh.loginfo("Invalid_msg");
            return;
    }      
    latest_msg.x = -1;  // Reset to default values
}
