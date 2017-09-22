/*
 * Jared and Matt PoE lab 2
 */

#include <Servo.h>
const int sensorPin = A0;
const int panServoPin = 9;
const int tiltServoPin = 10;

int theta=55; // Starting value for theta
int phi=60; // Starting value for phi

int thetaMult = 1;
int phiMult = 1;

bool isMoving = true;

Servo panServo;
Servo tiltServo;

void setup() {
  Serial.begin(9600);
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
}

void loop() {
  Serial.print(theta);
  Serial.print('x');
  Serial.print(phi);
  Serial.print('x');
  Serial.println(map(analogRead(sensorPin), 440, 190, 25, 60));

  theta += thetaMult;
  
  panServo.write(theta);
  tiltServo.write(phi);

  if (isMoving) {
    if (theta == 55 || theta == 145) {
      thetaMult = -thetaMult;
      phi += phiMult;
      if (phi == 120) {
        Serial.print('$');
        isMoving = false;
      }
    }
  }


  
  delay(50);
}


