#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

//   BNO055 Connections
//   ===========
//   CSCL --- A5
//   SDA ---- A4
//   VDD ---- 3.3-5V DC
//   GND ---- Common Ground


// ESC Connections
// ================
// Dark ----- GND
// Yellow --- 9


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55); //, 0x28);

Servo ESC; // Pretending our ESC is a servo

// Cycles PWM with intent to callibrate ESC
void cyclePWM() {
  for (int val = 0; val <= 50; val++) {
    Esc.write(val);
    delay(10);
  }
  delay(1000);
  for (int val = 50; val >= 0; val--) {
    ESC.write(val);
    delay(10);
  }
  delay(1000);
}


void setup() {
  Serial.begin(9600);
  if(!bno.begin()) {
    Serial.println("BNO055 not detected! Quitting!");
    exit();
  }
  bno.setExtCrystalUse(true);

  ESC.attach(9); //,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  cyclePWM();
  Serial.println("Setup finished!");
}

void loop() {

  // store sensor data in "event"
  sensors_event_t event;
  bno.getEvent(&event);

  float vel = event.gyro.x;
  float pos = event.orientation.roll

  float target_pos = 0;

  int P = 0.1 * (target_pos - pos);
  int I = 0;
  int D = 0.1 * (event.gyro.x);

  ESC.write(P + I + D);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
