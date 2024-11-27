#include "gurumi.h"



volatile long encoder1_pos = 0;
volatile long encoder2_pos = 0;

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

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), readEncoder2, CHANGE);
  
  analogWrite(MOTOR1_PWM,200);
  analogWrite(MOTOR2_PWM,200);

}

void loop() {

  Serial.print("motor1 : ");
  Serial.println(encoder1_pos);
  Serial.print("motor2 : ");
  Serial.println(encoder2_pos);

  delay(500);

  uint32_t t = millis();
  updateTime();

  nh.spinOnce();
  waitForSerialLink(nh.connected());

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
