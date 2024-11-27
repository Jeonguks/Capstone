#include "gurumi.h"



volatile long encoder1_pos = 0;
volatile long encoder2_pos = 0;

int motor1_pwm = 200; //initialize motor speed PWM
int motor2_pwm = 200;


const int ENCODER_ERROR_THRESHOLD = 5;
const int PWM_ADJUSTMENT = 20;


String current_led_command = "";  

unsigned long last_time = 0;
const unsigned long interval = 1000/15;

void readEncoder1(){
  if(digitalRead(ENCODER1_A) == digitalRead(ENCODER1_B)){
    encoder1_pos++;
  }else{     
    encoder1_pos--;
  }
}
void readEncoder2(){
  if(digitalRead(ENCODER2_A) == digitalRead(ENCODER2_B)){
    encoder2_pos--;
  }else{
    encoder2_pos++;
  }
}




void setup() {
  Serial.begin(57000);

  nh.initNode();

  nh.getHardware()->setBaud(115200);

  nh.subscribe(vehicle_control_sub); //add HERE

  nh.subscribe(led_command_sub); //add HERE

  pinMode(LED_WORKING_CHECK, OUTPUT);


  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT); 
  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);
  
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);

  pinMode(MOTOR1_DIR1, OUTPUT);
  pinMode(MOTOR1_DIR2, OUTPUT);
  pinMode(MOTOR2_DIR1, OUTPUT);
  pinMode(MOTOR2_DIR2, OUTPUT);

  pinMode(15, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(15,HIGH);
  digitalWrite(14,LOW);
  digitalWrite(13,HIGH);
  digitalWrite(12,LOW);


  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), readEncoder2, CHANGE);
  
  analogWrite(MOTOR1_PWM,200);
  analogWrite(MOTOR2_PWM,200);

}

void loop() {


  unsigned long current_time = millis();

  if(current_time - last_time >= interval){
    last_time = current_time;
  }
  
  updateTime();

  nh.spinOnce();
  
  waitForSerialLink(nh.connected());
  
  adjustMotorSpeeds();
  
  updateLEDStatus();
}

void updateLEDStatus(){
  if(current_led_command == "STANDBY"){
    digitalWrite(14,HIGH);
    digitalWrite(13,LOW);
    digitalWrite(12,HIGH);
    delay(500);
    digitalWrite(13,HIGH);
    digitalWrite(12,HIGH);
    delay(500);
  }else if(current_led_command == "GO"){
     digitalWrite(15,HIGH);
     digitalWrite(14,HIGH);
     digitalWrite(13,LOW);
     digitalWrite(12,HIGH);
  }else if(current_led_command == "HAND" || current_led_command == "WAITING"){
     digitalWrite(15,HIGH);
     digitalWrite(14,LOW);
     digitalWrite(13,HIGH);
     digitalWrite(12,LOW);
     delay(500);
     digitalWrite(14,HIGH);
     digitalWrite(13,HIGH);
     digitalWrite(12,HIGH);
     delay(500);
  }else if(current_led_command == "FOLLOWME"){
     digitalWrite(15,HIGH);
     digitalWrite(14,LOW);
     digitalWrite(13,HIGH);
     digitalWrite(12,LOW);
  }else{
    digitalWrite(15,HIGH);
    digitalWrite(14,LOW);
    digitalWrite(13,HIGH);
    digitalWrite(12,LOW);
  }
  
}

void adjustMotorSpeeds(){
  long encoder_diff = encoder1_pos - encoder2_pos;

  if(abs(encoder_diff)> ENCODER_ERROR_THRESHOLD){
    if(encoder_diff > 0){
      motor1_pwm = max(0,motor1_pwm - PWM_ADJUSTMENT);
      motor2_pwm = min(255,motor2_pwm - PWM_ADJUSTMENT);
    }else if(encoder_diff < 0){
      motor1_pwm = max(0,motor2_pwm - PWM_ADJUSTMENT);
      motor2_pwm = min(255,motor1_pwm + PWM_ADJUSTMENT);
    }

   analogWrite(MOTOR1_PWM, motor1_pwm);
   analogWrite(MOTOR2_PWM, motor2_pwm);

   motor_status_msg_motor1.data = motor1_pwm;
   motor_status_msg_motor2.data = motor2_pwm;
   motor_status_pub_motor1.publish(&motor_status_msg_motor1);
   motor_status_pub_motor2.publish(&motor_status_msg_motor2);
  }
}

void led_callback(const std_msgs::String& msg){
  current_led_command = msg.data; 
  updateLEDStatus();
} 



void control_callback(const std_msgs::String& msg){
  String command = msg.data;

  if(command == "REVERSE"){
    
    digitalWrite(MOTOR1_DIR1,LOW); //REVERSE
    digitalWrite(MOTOR1_DIR2,HIGH);
    digitalWrite(MOTOR2_DIR1,LOW);
    digitalWrite(MOTOR2_DIR2,HIGH);
    
  } else if(command =="FORWARD"){
    
    digitalWrite(MOTOR1_DIR1,HIGH); //FORWARD
    digitalWrite(MOTOR1_DIR2,LOW);
    digitalWrite(MOTOR2_DIR1,HIGH);
    digitalWrite(MOTOR2_DIR2,LOW);
    
  } else if(command == "RIGHT"){
    
    digitalWrite(MOTOR1_DIR1,LOW); //RIGHT
    digitalWrite(MOTOR1_DIR2,HIGH);
    digitalWrite(MOTOR2_DIR1,HIGH);
    digitalWrite(MOTOR2_DIR2,LOW);
    
  } else if(command == "LEFT"){
    
    digitalWrite(MOTOR1_DIR1,HIGH); //LEFT
    digitalWrite(MOTOR1_DIR2,LOW);
    digitalWrite(MOTOR2_DIR1,LOW);
    digitalWrite(MOTOR2_DIR2,HIGH);

  } else{
    
    digitalWrite(MOTOR1_DIR1,LOW); //Break
    digitalWrite(MOTOR1_DIR2,LOW);
    digitalWrite(MOTOR2_DIR1,LOW);
    digitalWrite(MOTOR2_DIR2,LOW);
    
  }
}

void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

ros::Time rosNow()
{
  return nh.now();
}
