#include <Arduino.h>

#define enA 11
#define in1 12
#define in2 13

int speed = 255; //0 to 255
bool rotateCCW = false; 
int d = 1000;
const int button_pin = 7; 
int button_state; 

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); //Motor driver pins 

  if (rotateCCW){
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH); 
  } else{
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW); 
  }
  //Set inital rotation direction
  
  analogWrite(enA, speed); // Set the speed 

  //pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(button_pin, INPUT_PULLUP);
  //Serial.begin(9600); 
}

void loop() {
  //Serial.println(digitalRead(button_pin)); 
  /*
  button_state = digitalRead(button_pin);
  if (button_state == HIGH){
    digitalWrite(LED_BUILTIN, HIGH);
  } else{
    digitalWrite(LED_BUILTIN, LOW); 
  }
  */
}

