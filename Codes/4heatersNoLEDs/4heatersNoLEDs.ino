#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <SPI.h>

// Pins Defined
#define heaterPin1 4 // Pin 2, PD4, C1D, Heater J2
#define heaterPin2 5 // Pin 9, PD5, C2D, Heater J3
#define heaterPin3 6 //Pin 10, PD6, C3D, Heater J4
#define heaterPin4 7 //Pin 11, PD7, C4D, Heater J5

//#define indicateGoodPin 8
//#define indicateHeatPin 6
//#define indicateDisablePin 5
//#define indicateErrorPin 4

#define disablePin 2

#define pauseTime 1000

// Sensor Configuations. LIMITS WILL CHANGE!!!!!!!!!!!!!!!!!!!!!!!!!!!!
const int addr[] = {0x18, 0x19, 0x1B, 0x1C};
const int control[] = {1, 1, 1, 1};
const float limits[] = {27.0, 27.0, 27.0, 27.0}; 
const int heaterPins[] = {heaterPin1, heaterPin2, heaterPin3, heaterPin4};

// Other Initializations
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
const int addrCount = sizeof(addr)/sizeof(addr[0]);
float temps[addrCount];
float temp;
int disabled;

int addrBad[addrCount];
int newAddr[addrCount];
//const int addrOtherUse[]; = {2, 3, 4, 1};

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

void setup() {
  // Heater Toggle Setup
  pinMode(heaterPin1, OUTPUT);
  pinMode(heaterPin2, OUTPUT);
  pinMode(heaterPin3, OUTPUT);
  pinMode(heaterPin4, OUTPUT);

  digitalWrite(heaterPin1, 0);
  digitalWrite(heaterPin2, 0);
  digitalWrite(heaterPin3, 0);
  digitalWrite(heaterPin4, 0);

  // Setup Disable Pin
  pinMode(disablePin, INPUT);

  // Communication Setup
  Serial.begin(9600);

  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
  tempsensor.setResolution(3);

}

void loop() {
  // INDICATE GOOD, this happens every loop.
  //Serial.println("Indicate good.");
  //delay(pauseTime/2);
  //digitalWrite(indicateGoodPin, 1);
  //delay(pauseTime/2);
  //digitalWrite(indicateGoodPin, 0);
  delay(pauseTime);

  // Reset other addr's for contingency.
  for(int i = 0; i < addrCount; i++){
    newAddr[i] = addr[i];
    addrBad[i] = 0;
  }

  // Check if a sensor isn't working.
  for(int i = 0; i < addrCount; i++){
    if(!tempsensor.begin(addr[i])) {
      Serial.print("COULD NOT CONNECT to MCP9808 on address: "); Serial.println(addr[i]);
      addrBad[i] = 1;
      //newAddr[i] = addr[addrOtherUse[i]];
    }
  }

  // Assign broken sensor to instead use next working sensor.
  for(int i = 0; i < addrCount; i++){
    if(addrBad[i]) {
      for(int j = 0; j < addrCount; j++){
        if(!addrBad[j]) {
          newAddr[i] = addr[j];
          Serial.print("Address "); Serial.print(addr[i]); Serial.print(" now uses temp from address: "); Serial.println(addr[j]); 
          j = 4; // End for loop.
        }
      }
    }
  }

  // READ TEMPERATURES
  for(int i = 0; i < addrCount; i++){
    temps[i] = getTemp(newAddr[i]);
  }

  // Set Heater Output
  disabled = !digitalRead(disablePin);
  // Powered (1) means enabled, unpowered (0) means disabled.
  // So "disabled" being (1) means disable is active, so turn heaters off.

  if(!disabled){ // If enabled (disabled = 0).
    //digitalWrite(indicateDisablePin, 0);
    for(int i = 0; i < addrCount; i++){
      if(control[i]){ // Checks if temp sensor is set to be used.
        if(temps[i] > limits[i]){ // Checks if temperature is above limit.
          digitalWrite(heaterPins[i], 0);; // Sets the heater to STOP.
        }else
          digitalWrite(heaterPins[i], 1); // Sets the heater to HEAT.
      }
    }
  }else{ // If disabled (disabled = 1).
    for(int i = 0; i < addrCount; i++){
      digitalWrite(heaterPins[i], 0);; // Sets all the heaters to stop (disabled).
    }
  }
}
