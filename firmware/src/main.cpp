#include <Arduino.h>

//A motor is the drain, B motor is the yield, C motor is the input water source   

#define enA 5
#define enB 6
#define enC 11 

#define inA1 7
#define inA2 8

#define inB1 9
#define inB2 10

#define inC1 12
#define inC2 2

void initMotorDriver(int en, int in1, int in2, int speed, bool rotCCW); 

void stopMotor(int in1, int in2); 

void startMotor(int in1, int in2, bool rotCCW);

int speedA = 30;
int speedB = 255;
int speedC = 255; 
//Speed between 0 to 255

bool rotA = false; 
bool rotB = false; 
bool rotC = true; 
//Does the motor spin counterclockwise?

int d = 1000;
const int button_pin = 4; 
int button_state; 
//Button Parameters 

void setup() {
  //initMotorDriver(enA, inA1, inA2, speedA, rotA); 
  //initMotorDriver(enB, inB1, inB2, speedB, rotB); 
  initMotorDriver(enC, inC1, inC2, speedC, rotC); 

  stopMotor(inA1, inA2);
  stopMotor(inB1, inB2);
  stopMotor(inC1, inB2); 

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);
  //Serial.begin(9600); 
}

void loop() {
  //Serial.println(digitalRead(button_pin)); 

  button_state = digitalRead(button_pin);
  if (button_state == HIGH){
    digitalWrite(LED_BUILTIN, HIGH);
    //startMotor(inA1, inA2, rotA);
    //startMotor(inB1, inB2, rotB); 
    startMotor(inC1, inC2, rotC);
  } else{
    digitalWrite(LED_BUILTIN, LOW); 
    stopMotor(inA1, inA2);
    stopMotor(inB1, inB2);
    stopMotor(inC1, inC2);
  }
  delay(1000); 
  //Debounce 
  
}

void initMotorDriver(int en, int in1, int in2, int speed, bool rotCCW){
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  //Motor driver pins 

  startMotor(in1, in2, rotCCW); 

  analogWrite(en, speed); 
  // Set the speed 
}

void stopMotor(int in1, int in2){
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);
  //Stop motors 
}

void startMotor(int in1, int in2, bool rotCCW){
  if (rotCCW){
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH); 
  } else{
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW); 
  }
  //Set rotation direction
}
