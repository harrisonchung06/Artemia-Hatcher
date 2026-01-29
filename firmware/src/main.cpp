#include <Arduino.h>
#include <Wire.h>
#include <LowPower.h>

/*
A motor is the yield 
B motor is the drain
C motor is the input water source   
*/

//Define motor driver pinout
#define enA 5
#define enB 6
#define enC 11 

#define inA1 7
#define inA2 8

#define inB1 9
#define inB2 10

#define inC1 12
#define inC2 4

//Define button pinout 
#define BUTTON 3

//Define clock pinout 
#define CLK_INT 2 
#define CLK_ADD 0x68
#define CLK_TIME_REG 0x00
#define CLK_ALARM1_REG 0x07
#define CLK_CONTROL_REG 0x0E
#define CLK_STATUS_REG 0x0F

//Prototype functions 

void initMotorDriver(int en, int in1, int in2, int speed, bool rotCCW); 

void stopMotor(int in1, int in2); 

void startMotor(int in1, int in2, bool rotCCW);

//Timer function in minutes
void sleepTimer(int secs, int mins, int hours); 

void setClockZero();

void setAlarm1(byte seconds, byte minutes, byte hours);

void clearAlarm1(); 

void onInterrupt();

void onInterruptButton();

void unarmAlarm1();

void armAlarm1();

uint8_t readRegister(uint8_t reg);

void writeRegister(uint8_t reg, uint8_t val);

byte decToBcd(byte data); 

byte bcdToDec(byte data); 

//Speed between 0 to 255
int speedA = 255;
int speedB = 255;
int speedC = 255; 

//Does the motor spin counterclockwise?
bool rotA = false; 
bool rotB = true; 
bool rotC = false; 
bool rotB = true; 
bool rotC = false; 

//Button Parameter 
int buttonState;

//Volume in liters 
float v = 1.33;
//Liters per minute 
float flowRate = 0.06;

void setup() {
  Serial.begin(9600); 
  Wire.begin();
  //Motors 
  initMotorDriver(enA, inA1, inA2, speedA, rotA); 
  initMotorDriver(enB, inB1, inB2, speedB, rotB); 
  initMotorDriver(enC, inC1, inC2, speedC, rotC); 

  stopMotor(inA1, inA2);
  stopMotor(inB1, inB2);
  stopMotor(inC1, inB2); 

  pinMode(LED_BUILTIN, OUTPUT);
  
  //Interrupts
  pinMode(CLK_INT, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(CLK_INT), onInterrupt, FALLING); 

  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON), onInterruptButton, LOW); 

  setClockZero(); 
}

void loop() {
  buttonState = digitalRead(BUTTON);
  if (buttonState == HIGH){
    digitalWrite(LED_BUILTIN, HIGH);

    startMotor(inB1, inB2, rotB);
    sleepTimer(5,0,0);
    stopMotor(inB1, inB2);
    
    startMotor(inB1, inB2, rotB);
    sleepTimer(0,10,0); //Setup initial conditions (all drained)
    stopMotor(inB1, inB2); 

    startMotor(inC1, inC2, rotC);
    sleepTimer(0,9,0); //Fill all 
    stopMotor(inC1, inC2);

    sleepTimer(0,0,30);  //Wait 30 hours 

    startMotor(inB1, inB2, rotB);
    sleepTimer(0,10,0); //Drain all
    stopMotor(inB1, inB2); //Stop Drain 

    startMotor(inC1, inC2, rotC); 
    sleepTimer(0,1,0); //Fill

    startMotor(inB1,inB2, rotB); 
    sleepTimer(0,10,0); //Flush 

    stopMotor(inC1, inC2); //Stop flush
    stopMotor(inB1, inB2); 

    startMotor(inA1, inA2, rotA); //Yield 
    sleepTimer(0,3,0); //sleepTimer(0,1,0);
    stopMotor(inA1, inA2);

    //Wait for manual reset 
    sleepTimer(0,0,999); 
    
  }
}

//Motor Functions
void initMotorDriver(int en, int in1, int in2, int speed, bool rotCCW){
  //Motor driver pins 
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 

  startMotor(in1, in2, rotCCW); 
  // Set the speed 
  analogWrite(en, speed); 
  
}

void stopMotor(int in1, int in2){
  //Stop motors
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);
}

void startMotor(int in1, int in2, bool rotCCW){
  //Set rotation direction
  if (rotCCW){
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH); 
  } else{
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW); 
  } 
}

//Timer Functions
void sleepTimer(int secs, int mins, int hours){
  setClockZero();
  setAlarm1(secs, mins, hours); 
  //Low power 
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
}

//Set microcontroller clock to zero 
void setClockZero(){ 
  Wire.beginTransmission(CLK_ADD);
  
  Wire.write(CLK_TIME_REG);
  //Write seconds 
  Wire.write(decToBcd(0));
  //Write minutes
  Wire.write(decToBcd(0));
  //Write Hours
  Wire.write(decToBcd(0));
   
  Wire.endTransmission(); 
}

void setAlarm1(byte second, byte minute, byte hour){
  //Ignore DWDY matching (Mask bit)
  byte dydw = 0b10000000; 
  
  //Unarm and clear alarm
  unarmAlarm1(); 
  clearAlarm1();  
 
  //Convert to Bit
  second = decToBcd(second);
  minute = decToBcd(minute);
  hour = decToBcd(hour); 
  
  //Bitwise operator to enable matching (Mask bit)
  second &= 0b01111111;
  minute &= 0b01111111;
  hour &= 0b01111111;
  
  //Write to alarm register
  Wire.beginTransmission(CLK_ADD); //Begin Transmission
  Wire.write(CLK_ALARM1_REG);
  Wire.write(second);
  Wire.write(minute);
  Wire.write(hour);
  Wire.write(dydw);
  Wire.endTransmission();

  //Arm alarm 
  armAlarm1(); 
  
}

void clearAlarm1(){
  uint8_t val;
  val = readRegister(CLK_STATUS_REG);
  val &= 0b11111110;
  writeRegister(CLK_STATUS_REG, val);
}

void armAlarm1(){
  uint8_t val;
  val = readRegister(CLK_CONTROL_REG); 
  val |= 0b00000001; //A1IE
  val |= 0b00000100; //INTCN
  writeRegister(CLK_CONTROL_REG, val);
}

void unarmAlarm1(){
  uint8_t val;
  val = readRegister(CLK_CONTROL_REG); 
  val &= 0b11111110;
  writeRegister(CLK_CONTROL_REG, val); 
}

uint8_t readRegister(uint8_t reg){
  uint8_t val;
  Wire.beginTransmission(CLK_ADD);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(CLK_ADD, 1);
  val = Wire.read();
  return val; 
}

void writeRegister(uint8_t reg, uint8_t val){
  Wire.beginTransmission(CLK_ADD);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

byte decToBcd(byte data){
  return ( (data/10*16) + (data%10) );
}
byte bcdToDec(byte data){
  return ( (data/16*10) + (data%16) );
} 

//Service Functions
void onInterrupt(){
}

void onInterruptButton(){
  digitalWrite(LED_BUILTIN, LOW); 
  stopMotor(inA1, inA2);
  stopMotor(inB1, inB2);
  stopMotor(inC1, inC2);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF,BOD_OFF);
}


