#include <Servo.h>

// SIMPLE MOTOR-SPIN Program

Servo ESC; // Pretending our ESC is a servo


void setup() {
  Serial.begin(9600);
  
  // Attach the ESC on pin 9
  ESC.attach(9);

  ESC.writeMicroseconds(1500);
  delay(4000);
}

void loop() {

  Serial.println("Enter PWM microsecond value 1000 to 2000, 1500 to stop");
  while (Serial.available() == 0);
  int pwm = Serial.parseInt();
  ESC.writeMicroseconds(pwm); // Send signal to ESC.
  
}
