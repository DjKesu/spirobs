#include <ESP32Servo.h>

Servo myServo1;
Servo myServo2;

void setup() {
  myServo1.attach(23);
  myServo2.attach(13);
}

void loop() {
  // Rotate myServo1 forward at full speed
  myServo1.writeMicroseconds(2500);
  delay(2000); // Rotate for 2 seconds

  // Stop myServo1
  myServo1.writeMicroseconds(1500);
  delay(1000); // Pause for 1 second

  // Rotate myServo1 backward at full speed
  myServo1.writeMicroseconds(500);
  delay(2000); // Rotate for 2 seconds

  // Stop myServo1
  myServo1.writeMicroseconds(1500);
  delay(1000); // Pause for 1 second

  // Repeat the same for myServo2
  myServo2.writeMicroseconds(2500);
  delay(2000);

  myServo2.writeMicroseconds(1500);
  delay(1000);

  myServo2.writeMicroseconds(1500);
  delay(2000);

  myServo2.writeMicroseconds(500);
  delay(1000);
}
