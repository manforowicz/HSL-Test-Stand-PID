#include <Servo.h>

// SIMPLE MOTOR-SPIN Program

Servo ESC; // Pretending our ESC is a servo

// Calibrate ESC by plugging in battery when
// MAX_PWM is being outputted.
void calibrateESC() {
  ESC.writeMicroseconds(MAX_PWM);
  delay(8000);
  ESC.writeMicroseconds(MIN_PWM);
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  
  // Attach the ESC on pin 9
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

  ESC.writeMicroseconds(1500);
  delay(1000);
}

void loop() {

}
