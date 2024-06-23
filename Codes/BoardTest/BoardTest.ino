// Define pin number
const int pin = 3;  // D4 on the board

void setup() {
  // Set the digital pin as an output
  pinMode(pin, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Turn the pin on (HIGH is the voltage level)
  digitalWrite(pin, HIGH);
  // Wait for a second (1000 milliseconds)
  delay(1000);
  // Turn the pin off by making the voltage LOW
  digitalWrite(pin, LOW);
  // Wait for a second
  delay(1000);

  Serial.println("hello world");
  
}
