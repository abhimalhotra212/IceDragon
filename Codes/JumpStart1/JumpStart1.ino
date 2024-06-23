// Ice Dragon
// Jump Start Circuit

#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <SPI.h>

// Define Pins
#define Check5Volt 14
#define Check6Volt 15
#define Jump5Volt 4
#define Jump6Volt 5
#define testLight 3 

// waitTime for each loop of void loop, waitTimeShort for when jump occurs.
#define waitTime 500
#define waitTimeShort 250

// "analogRead" gives values form 1 to 1023, according to 0 to 5 volts respectively.
// These vals define what we should be using (2 volts and 2.5 volts).
#define volt5Num 409
#define volt6Num 512


void setup() {
  // Begin Communication
  Serial.begin(9600);

  // Setup Inputs/Outputs
  pinMode(Check5Volt, INPUT);
  pinMode(Check6Volt, INPUT);
  pinMode(Jump5Volt, OUTPUT);
  pinMode(Jump6Volt, OUTPUT);

  pinMode(testLight, OUTPUT);

  digitalWrite(Jump5Volt,1);
  digitalWrite(Jump6Volt,1);

  // Setup run, medium paced flashing
  for(int i = 0; i < 12; i++){
    digitalWrite(testLight, 1);
    delay(175);
    digitalWrite(testLight, 0);
    delay(175);
  }
}

void loop() {
  Serial.println("Void loop starting!");

  // Check to jump 5 volt
  Serial.print("Analog read 5 Volts: "); Serial.println(analogRead(Check5Volt));
  if (analogRead(Check5Volt) < volt5Num){
    digitalWrite(Jump5Volt,0);
    delay(waitTimeShort);
    digitalWrite(Jump5Volt,1);
    delay(waitTimeShort);
    Serial.println("Jumped 5 volts!");

    // 5 volt jumped, slow flashing
    for(int i = 0; i < 5; i++){
      digitalWrite(testLight, 1);
      delay(250);
      digitalWrite(testLight, 0);
      delay(250);
    }
  }

  // Check to jump 6 volt
  Serial.print("Analog read 6 Volts: "); Serial.println(analogRead(Check6Volt));
  if (analogRead(Check6Volt) < volt6Num){
    digitalWrite(Jump6Volt,0);
    delay(waitTimeShort);
    digitalWrite(Jump6Volt,1);
    delay(waitTimeShort);
    Serial.println("Jumped 6 volts!");

    // 6 volt jumped, fast flashing
    for(int i = 0; i < 20; i++){
      digitalWrite(testLight, 1);
      delay(100);
      digitalWrite(testLight, 0);
      delay(100);
    }
  }

  // Quick Pause
  delay(waitTime);
}
