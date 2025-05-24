#include <Arduino.h>
uint8_t motor_pins[4]={14,15,16,17};
//Ultrasonic Pins
int trigPins[4]={5,19,22,25};
int echoPins[4]={18,21,23,26};
//Joystick Pins
int joyPins[2]={36,39};
bool calibrated=false;
void PWMsetup(){
  Serial.begin(115200);
  for(int i=0;i<4;i++){
ledcSetup(i,50,16);
ledcAttachPin(motor_pins[i],i);
  }
  analogReadResolution(12);
  if(!calibrated){
    Serial.println("Starting ESC calibration...");
    //Max Throttle
    for(int i=0;i<4;i++){
    ledcWrite(i,7000);
    }
    //Min throttle
    for(int i=0;i<4;i++){
      ledcWrite(i,3000);
      }
      calibrated=true;
      Serial.println("Calibration Successful!");
  }
}
void Setup(){
  for(int i=0;i<4;i++){
    pinMode(echoPins[i],INPUT);
    pinMode(trigPins[i],OUTPUT);
    for(int i=0;i<2;i++){
      pinMode(joyPins[i],INPUT);
    }
  }
}
long distanceRead(int trigPin,int echoPin){
digitalWrite(trigPin,LOW);
delayMicroseconds(2);
digitalWrite(trigPin,HIGH);
delayMicroseconds(10);
long duration=pulseIn(echoPin,HIGH,30000);
if(duration==0)
return -1;
else
return duration/58;
}
void loop(){
  int joyRollVal=analogRead(33);
  int roll=map(joyRollVal,0,4095,-200,+200);
  int joyPitchVal=analogRead(32);
  int pitch=map(joyPitchVal,0,4095,-200,200);
  int joyThrottleVal=analogRead(36);
int joyYawVal=analogRead(39);
int throttle=map(joyThrottleVal,0,4095,3000,7000);
int yaw=map(joyYawVal,0,4095,-200,200);
long distances[4];
  for(int i=0;i<4;i++){
  distances[i]=distanceRead(trigPins[i],echoPins[i]);
    if(distances[i]=-1)
    distances[i]=300; //max distance if no echo
  }
//Collision Avoidance Logic
int adjThrottle=throttle;
int adjYaw=yaw;
    if(distances[0]<30){
  adjThrottle=3000;
  pitch=0;
  roll=0;
  }
    if(distances[2]<30){
  adjYaw=-100;
  pitch=0;
  roll=0;
  }
   if(distances[3]<30){
  adjYaw=+100;
  pitch=0;
  roll=0;
  }
  //Motor Mixing
  yaw=adjYaw;
  throttle=adjThrottle;
  int motorspeeds[4];
  motorspeeds[0]=constrain(throttle+yaw+pitch+roll,3000,7000);
  motorspeeds[1]=constrain(throttle-yaw+pitch-roll,3000,7000);
  motorspeeds[2]=constrain(throttle-yaw-pitch+roll,3000,7000);
  motorspeeds[3]=constrain(throttle+yaw-pitch-roll,3000,7000);
  for(int i=0;i<4;i++){
    ledcWrite(i,motorspeeds[i]);
  }
  }
  
