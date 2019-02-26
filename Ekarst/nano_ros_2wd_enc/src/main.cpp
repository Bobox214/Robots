#include <Arduino.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

std_msgs::Int32 msgEncoder1;
ros::Publisher pubEncoder1("Encoder1",&msgEncoder1);
base* Base;

char msg[128];

void cmdVelCb( const geometry_msgs::Twist& cmd_vel_msg){
    motors_timer = millis() + motors_timeout;
    nh.loginfo("Motor cmd_vel received");
    float v = cmd_vel_msg.linear.x;
    float w = cmd_vel_msg.angular.z;
    float vL = v+(w*baseWidth)/2;
    float vR = v-(w*baseWidth)/2;
    float vLrpm = vL*30/(PI*wheelRadius);
    float vRrpm = vR*30/(PI*wheelRadius);
    sprintf(msg,"Current speed 1: %d vs %d 2: %d vs %d ; v:%d w:%d",
        int(1000*Encoder1.getCurrentSpeed())
    ,   int(1000*-vRrpm)
    ,   int(1000*Encoder2.getCurrentSpeed())
    ,   int(1000*vLrpm)
    ,   int(1000*v)
    ,   int(1000*w)
    );
    nh.loginfo(msg);
    Encoder1.runSpeed(-vRrpm);
    Encoder2.runSpeed(vLrpm);
    motors_timer = millis()+motors_timeout;
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCb);

void waitRosConnection() {

    float pidConstants[3];

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

    Encoder1.setSpeedPid(pidConstants[0],pidConstants[1],pidConstants[2]);
    Encoder2.setSpeedPid(pidConstants[0],pidConstants[1],pidConstants[2]);

    Encoder1.setPulsePos(0);
    Encoder2.setPulsePos(0);

    motors_timer = 0;
}

const int BTN   = 19;

bool pressed = 0;
int8_t incr = 1;
// 610 par tour
void reset() {
    digitalWrite(LED_BUILTIN,0);
}
void setup() {
    Serial.begin(9600);
    pinMode(BTN  , INPUT );
    reset();
    digitalWrite(LED_BUILTIN,1);
    while (!Serial) {}
    digitalWrite(LED_BUILTIN,0);
}

void loop() {
    if (digitalRead(BTN)) {
        pressed = !pressed;
        if (!pressed)
            reset();
        if (pressed) {
            digitalWrite(LED_BUILTIN,1);
        }
        delay(200);
    }
}
