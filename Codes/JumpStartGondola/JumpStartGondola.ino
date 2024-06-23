// Ice Dragon
// Jump Start Circuit
// For gondola board
// Constantly output jump pins so jumps never occur.

#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <SPI.h>

// Define Pins for shortcutting
#define Jump5Volt 4
#define Jump6Volt 5
#define testLight 3 

void setup() {
  // Begin Communication
  Serial.begin(9600);

  // Setup pins as being either Inputs/Outputs
  pinMode(Jump5Volt, OUTPUT);
  pinMode(Jump6Volt, OUTPUT);
  pinMode(testLight, OUTPUT);

  // Initially setup output pins to be what is needed for code.
  // In this case (gondola board), the outputs are sent to be constantly on so that no jumping occurs.
  digitalWrite(Jump5Volt,1);
  digitalWrite(Jump6Volt,1);

  // Setup run, medium paced flashing. This is just to indicate the program started.
  for(int i = 0; i < 6; i++){
    digitalWrite(testLight, 1);
    delay(175);
    digitalWrite(testLight, 0);
    delay(175);
  }
}

void loop() {
  // Flashing just to show that the arduino is operating. 
  digitalWrite(testLight, 1);
    delay(1000);
    digitalWrite(testLight, 0);
    delay(1000);
}
