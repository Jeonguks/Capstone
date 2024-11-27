#ifndef GURUMI_H_
#define GURUMI_H_


#include <ros.h>
#include <ros/time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <math.h>

#define LED_WORKING_CHECK 13
#define ENCODER1_A 2 //motor1 encoder A interrupt
#define ENCODER1_B 3 //motor1 encoder B
#define ENCODER2_A 4 //motor2 encoder A intterupt
#define ENCODER2_B 5 //motor2 encoder B 

#define MOTOR1_PWM 10 
#define MOTOR2_PWM 11 

#define MOTOR1_DIR1 6 
#define MOTOR1_DIR2 7 
#define MOTOR2_DIR1 8 
#define MOTOR2_DIR2 9

void control_callback(const std_msgs::String& msg);

void EncoderInit(void);
void wheelSpeed(void);


void waitForSerialLink(bool isConnected);

ros::Time rosNow(void);
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

ros::Subscriber<std_msgs::String> vehicle_control_sub("/vehicle_control", &control_callback);


#endif
