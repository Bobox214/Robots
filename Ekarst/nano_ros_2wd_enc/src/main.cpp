#include <Arduino.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

typedef ros::NodeHandle_<ArduinoHardware, 1, 1,128,128> NodeHandle_t;

#include <Base.h>

#define TICKS_PER_ROTATION 610

NodeHandle_t nh;

Base  base;

const float pidKp            = 20.0;
const float pidKi            = 50.0;
const float pidKd            = 5.0;
const float baseWidth        = 0.34;
const float wheelRadius      = 0.075;
const int   ticksPerRotation = 610;

uint32_t  motors_timer;    // Handles stopping motors after certain time without message
int       motors_timeout   = 1000;

bool directCmd = false;  // When set to True, cmd_vel is directly converted to Motor PWM
                         // When set to False, cmd_vel is converted to motor speed with a PID
void cmdVelCb( const geometry_msgs::Twist& cmd_vel_msg){
    motors_timer = millis() + motors_timeout;
    float v = cmd_vel_msg.linear.x;
    float w = cmd_vel_msg.angular.z;
    if (directCmd) {
       int pwmL = constrain(v*400-w*100,-255,255); 
       int pwmR = constrain(v*400+w*100,-255,255); 
       base.setMotorPwm(pwmL,pwmR);
    } else {
        base.setSpeed(v,w);
    }
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCb);

void waitRosConnection() {

    while (!nh.connected()) { nh.spinOnce(); };


    if (!nh.getParam("~direct_cmd", &directCmd)) {
        nh.loginfo("Using default value false for direct_cmd");
    } else
        nh.loginfo("Direct_cmd received");
    if (!nh.getParam("~motors_timeout", &motors_timeout)) {
        nh.loginfo("Using default value for motors_timeout");
    } else
        nh.loginfo("motors_timeout received");
    //nh.getParam("~debug", &debug);

    base.setParameters(baseWidth,wheelRadius,ticksPerRotation);
    base.setPIDs(pidKp,pidKi,pidKp);
    base.setNodeHandler(&nh);
    base.setDebug(true);
    base.stop();
    motors_timer = 0;
}

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
    delay(50);
}
