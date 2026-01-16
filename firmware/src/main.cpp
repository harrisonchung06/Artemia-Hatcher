#include <Arduino.h>
#include <Wire.h>
#include <ArduinoLowPower.h>

//A motor is the drain, B motor is the yield, C motor is the input water source   

//Define Pinout
#define enA 5
#define enB 6
#define enC 11 

#define inA1 7
#define inA2 8

#define inB1 9
#define inB2 10

#define inC1 12
#define inC2 3

#define CLK_INT 2 
#define CLK_ADD 0x68
#define CLK_TIME_REG 0x00
#define CLK_ALARM1_REG 0x07
#define CLK_CONTROL_REG 0x0E
#define CLK_STATUS_REG 0x0F

void initMotorDriver(int en, int in1, int in2, int speed, bool rotCCW); 

void stopMotor(int in1, int in2); 

void startMotor(int in1, int in2, bool rotCCW);

void sleepTimer(int secs, int mins, int hours); //Timer function in minutes

void setClockZero();

void setAlarm1(byte seconds, byte minutes, byte hours);

void clearAlarm1(); 

void onInterrupt();

void unarmAlarm1();

void armAlarm1();

uint8_t readRegister(uint8_t reg);

void writeRegister(uint8_t reg, uint8_t val);

byte decToBcd(byte data); 

byte bcdToDec(byte data); 

byte* getLocalTime();


int speedA = 30;
int speedB = 255;
int speedC = 255; 
//Speed between 0 to 255

bool rotA = false; 
bool rotB = true; 
bool rotC = false; 
//Does the motor spin counterclockwise?

int d = 1000;
const int button_pin = 4; 
int button_state; 
//Button Parameters 

float v = 1.33; //Volume in liters 
float flowRate = 0.06; //Liters per minute 

byte reg[3] = {0x00,0x01,0x02};
byte val[3]; 
byte* ptr; 

void setup() {
  Serial.begin(9600); 
  Wire.begin();

  initMotorDriver(enA, inA1, inA2, speedA, rotA); 
  initMotorDriver(enB, inB1, inB2, speedB, rotB); 
  initMotorDriver(enC, inC1, inC2, speedC, rotC); 

  stopMotor(inA1, inA2);
  stopMotor(inB1, inB2);
  stopMotor(inC1, inB2); 

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);

  pinMode(CLK_INT, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(CLK_INT), onInterrupt, FALLING); 
  LowPower.attachInterruptWakeup(CLK_INT, onInterrupt, FALLING); 

  setClockZero(); 
}

void loop() {
  //Serial.println(digitalRead(button_pin)); 
  button_state = digitalRead(button_pin);
  if (button_state == HIGH){
    digitalWrite(LED_BUILTIN, HIGH);
    startMotor(inC1, inC2, rotC);
    //Timer 9 minutes 
    delay(1000);
    stopMotor(inC1, inC2);
    //Wait 30 hours 
    delay(1000);
    //Drain open
    startMotor(inC1, inC2, rotC); // Flush
    //Timer 10 minutes 
    //Drain close 
    //Timer 3 minutes 
    delay(1000);
    stopMotor(inC1, inC2);
    startMotor(inB1, inB2, rotB); //Yield 
    //Timer 1 minutes
    delay(1000);
    stopMotor(inB1, inB2);
    
  } else{
    digitalWrite(LED_BUILTIN, LOW); 
    stopMotor(inA1, inA2);
    stopMotor(inB1, inB2);
    stopMotor(inC1, inC2);
  }
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
//Motor Functions

void onInterrupt(){
   unarmAlarm1();
   clearAlarm1(); 
}

void sleepTimer(int secs, int mins, int hours){
  setClockZero();
  setAlarm1(secs, mins, hours); 
  LowPower.deepSleep(); //Infinite deep sleep 
}

void setClockZero(){ //Set microcontroller clock to zero 
  Wire.beginTransmission(CLK_ADD);
  
  Wire.write(CLK_TIME_REG);
  Wire.write(decToBcd(0));
  //Write seconds 
  Wire.write(decToBcd(0));
  //Write minutes
  Wire.write(decToBcd(0));
  //Write Hours 
  Wire.endTransmission(); 
}

byte* getLocalTime() { //Time local to the microcontroller 
  for (int i = 0; i<3; i++){
    Wire.beginTransmission(CLK_ADD);
    Wire.write(reg[i]);
    Wire.endTransmission();
    Wire.requestFrom(CLK_ADD, 1); 
    val[i] = bcdToDec(Wire.read());
  }
  return val; 
}

byte decToBcd(byte data){
  return ( (data/10*16) + (data%10) );
}
byte bcdToDec(byte data){
  return ( (data/16*10) + (data%16) );
}

void setAlarm1(byte second, byte minute, byte hour){
  byte dydw = 0b10000000; 
  //Ignore DWDY matching (Mask bit)

  unarmAlarm1(); 
  clearAlarm1();  
  //Unarm and clear alarm

  second = decToBcd(second);
  minute = decToBcd(minute);
  hour = decToBcd(hour); 
  //Convert to Bit

  second &= 0b01111111;
  minute &= 0b01111111;
  hour &= 0b01111111;
  //Bitwise operator to enable matching (Mask bit)
  Wire.beginTransmission(CLK_ADD); 
  Wire.write(CLK_ALARM1_REG);
  Wire.write(second);
  Wire.write(minute);
  Wire.write(hour);
  Wire.write(dydw);
  Wire.endTransmission();
  //Write to alarm register 

  armAlarm1(); 
  //Arm alarm 
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

//Timer Functions 



