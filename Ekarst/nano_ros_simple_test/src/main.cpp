/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000 
//#define USE_USBCON 

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char msg[256];

void messageCb( const std_msgs::String& str_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
  strcpy(msg,str_msg.data);
}

ros::Subscriber<std_msgs::String> sub("message", &messageCb );

void setup()
{
  strcpy(msg,"Hello world!");
  digitalWrite(LED_BUILTIN,0);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  while (!nh.connected()) { nh.spinOnce(); };
  digitalWrite(LED_BUILTIN,1);
}

void loop()
{
  str_msg.data = msg;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
