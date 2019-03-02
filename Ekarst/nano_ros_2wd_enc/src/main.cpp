#include <Arduino.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <Base.h>

#define TICKS_PER_ROTATION 610

ros::NodeHandle nh;

//std_msgs::Int32 msgEncoder1;
//ros::Publisher pubEncoder1("Encoder1",&msgEncoder1);
Base  base;

uint32_t  motors_timer;    // Handles stopping motors after certain time without message
int     motors_timeout;

char msg[128];

void cmdVelCb( const geometry_msgs::Twist& cmd_vel_msg){
    motors_timer = millis() + motors_timeout;
    nh.loginfo("Motor cmd_vel received");
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

    if (!nh.getParam("~motors_timeout", &motors_timeout)) { 
       nh.loginfo("Using default values for motors_timeout");
       motors_timeout = 1000;
    }
    if (!nh.getParam("~pid", pidConstants, 3)) { 
       nh.loginfo("Using default values for pid");
       pidConstants[0] = 0.18;
       pidConstants[1] = 0;
       pidConstants[2] = 0;
    }

    if (!nh.getParam("~baseWidth", &baseWidth)) { 
       baseWidth = 0.17;
    }
    if (!nh.getParam("~wheelRadius", &wheelRadius)) { 
       wheelRadius = 0.032;
    }
    if (!nh.getParam("~ticksPerRotation", &ticksPerRotation)) { 
       ticksPerRotation = 610;
    }

    base.setParameters(baseWidth,wheelRadius,ticksPerRotation);
    base.setPIDs(pidConstants[0],pidConstants[1],pidConstants[2]);
    base.stop();
    motors_timer = 0;
}

const int BTN   = 19;

bool pressed = 0;
// 610 par tour

void reset() {
  digitalWrite(LED_BUILTIN,0);
  base.stop();
}

void setup() {
    pinMode(BTN  , INPUT );

    nh.initNode();
    nh.subscribe(cmdVelSub);
    digitalWrite(LED_BUILTIN,0);
    waitRosConnection();
    digitalWrite(LED_BUILTIN,1);
    pressed = 1;
}

void loop() {
    if (!nh.connected())

    if (digitalRead(BTN)) {
        pressed = !pressed;
        if (!pressed) {
          reset();
        }
        if (pressed) {
            digitalWrite(LED_BUILTIN,1);
        }
        delay(200);
    }

    if (motors_timer!=0 && millis()>motors_timer) {
        base.stop();
        motors_timer = 0;
        nh.loginfo("Motors have been stopped after timeout");
    }

    base.loop();
    nh.spinOnce();
}
