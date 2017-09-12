/*
 * Jared and Matt PoE lab 2
 */
 
const int sensorPin = A0;
const int servoPin = 3;

void setup() {
  Serial.begin(9600);
}

void loop() {
Serial.println(analogRead(sensorPin));
}
