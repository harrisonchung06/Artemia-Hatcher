#include <Arduino.h>

int d = 1000;
const int button_pin = 7; 

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);
  Serial.begin(9600); 
}

void loop() {
  Serial.println(digitalRead(button_pin)); 
}

