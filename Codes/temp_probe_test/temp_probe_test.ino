// Temperature Probe Test Code
// Matthew Law, 3 May 2024
// See Video I sent: https://youtu.be/10ZzGoNzfnw

// PROBE TEST CODE
// Heater 1 flashes on and off at room temperature if probe works properly (unless it is very cold, less than 20 C)
// By default, Probe 1 is tested
// Change "address" constant to test different ports
// Watch the video linked above

#include "Adafruit_MCP9808.h"

// Define pin number
const int pin = 3;  // D3 on the board
const int heaterPin = 4; // Heater pin 
const int address = 0x18; // Addresses are 0x18 for Probe 1, 0x19 for Probe 2, Ox1A for Probe 3, 0x1B for Probe 4

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

void setup() {
  // Set the digital pin as an output
  pinMode(pin, OUTPUT);
}

void loop() {
  int pauseTime = (getTemp(0x18)-20)*250;

  if(pauseTime < 10){
    pauseTime = 10;
  }
  
  delay(pauseTime);
  digitalWrite(heaterPin, 1);
  delay(pauseTime);
  digitalWrite(heaterPin, 0);
}

// Function to get the temperature from the thermister.
float getTemp(int funcAddr) {
  if(!tempsensor.begin(funcAddr)) {
    Serial.print("COULD NOT CONNECT to MCP9808 on address: "); Serial.println(funcAddr);
    return 5000.0; // This is just a number so big it forces heaters to turn off.
  }else {
    tempsensor.wake();
    float tempC = tempsensor.readTempC();
    tempsensor.shutdown();
    Serial.print("Connected and reading on address: "); Serial.println(funcAddr);
    Serial.print("Temp: "); Serial.println(tempC);
    return tempC;
  }
}
