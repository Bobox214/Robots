#include <Arduino.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <Base.h>

#define TICKS_PER_ROTATION 610

ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 128> nh;

Base  base;

uint32_t  motors_timer;    // Handles stopping motors after certain time without message
int     motors_timeout;

void cmdVelCb( const geometry_msgs::Twist& cmd_vel_msg){
    motors_timer = millis() + motors_timeout;
    float v = cmd_vel_msg.linear.x;
    float w = cmd_vel_msg.angular.z;
    base.setSpeed(v,w);
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCb);

void waitRosConnection() {

    float pidConstants[3];
    float baseWidth;
    float wheelRadius;
    int   ticksPerRotation;

    // Get Node parameters
    while (!nh.connected()) { nh.spinOnce(); };

    motors_timeout   = 1000;
    pidConstants[0]  = 20;
    pidConstants[1]  = 50;
    pidConstants[2]  = 5;
    baseWidth        = 0.34;
    wheelRadius      = 0.075;
    ticksPerRotation = 610;

    base.setParameters(baseWidth,wheelRadius,ticksPerRotation);
    base.setPIDs(pidConstants[0],pidConstants[1],pidConstants[2]);
    base.setNodeHandler(&nh);
    base.setDebug(true);
    base.stop();
    motors_timer = 0;
}

const int BTN   = 19;

// 610 par tour

void setup() {
    nh.initNode();
    nh.subscribe(cmdVelSub);
    digitalWrite(LED_BUILTIN,0);
    waitRosConnection();
    digitalWrite(LED_BUILTIN,1);
}

void loop() {

    if (motors_timer!=0 && millis()>motors_timer) {
        base.stop();
        motors_timer = 0;
        nh.loginfo("Motors have been stopped after timeout");
    }

    base.loop();
    nh.spinOnce();
    delay(100);
}
