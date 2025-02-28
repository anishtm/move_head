#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>


ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);


void messageCb(const geometry_msgs::Point32& msg) {
    uint8_t type = int(msg.z);
    int16_t position = int(msg.x);
    uint8_t angle = int(msg.y);
    char buffer[30];

 
    sprintf(buffer, "Type: %d, Pos: %d servo: %d", type, position, angle);
    str_msg.data = buffer;
           
    chatter.publish(&str_msg);
}

ros::Subscriber<geometry_msgs::Point32> sub("commands", messageCb);




void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(sub);

}

void loop() {
    nh.spinOnce();
    delay(1);
}
