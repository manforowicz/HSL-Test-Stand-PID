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
#define BNO055_SAMPLERATE_DELAY_MS (10)

#define STATIONARY_PWM (1500)

#define MAX_PWM (2000)

#define MIN_PWM (1000)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55); //, 0x28);

float target_pos;

Servo ESC; // Pretending our ESC is a servo

// Calibrate ESC by plugging in battery when
// MAX_PWM is being outputted.
void calibrateESC() {
  ESC.writeMicroseconds(MAX_PWM);
  delay(8000);
  ESC.writeMicroseconds(MIN_PWM);
  delay(1000);
}

// Displays orientation
void displaySensorDetails(sensors_event_t &event) {
  Serial.print("Orientation.roll "); Serial.println(event.orientation.roll);
  // Serial.print("Orientation.azimuth "); Serial.println(event.orientation.azimuth);
  Serial.print("Orientation.pitch "); Serial.println(event.orientation.pitch);
}

void setup() {
  Serial.begin(9600);
  
  // Set up the IMU
  if(!bno.begin()) {
    Serial.println("BNO055 not detected! Quitting!");
    exit(1);
  }
  bno.setExtCrystalUse(true);

  // Set up the ESC
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.writeMicroseconds(STATIONARY_PWM);

  // Set the target_pos to current position
  sensors_event_t event;
  bno.getEvent(&event);
  target_pos = event.orientation.roll;

  Serial.println("Setup finished!");

}

void loop() {
  // store sensor data in "event"
  sensors_event_t event;
  bno.getEvent(&event);

  float vel = event.gyro.x; // gyro gives velocity
  float pos = event.orientation.roll;

  float P = 0.5 * (pos - target_pos);
  float I = 0; // set to zero for now
  float D = -0.0 * (vel); // set to zero for now

  float pwm_out = STATIONARY_PWM + P + I + D;

  // constrain pwm to range
  pwm_out = constrain(write_val, MIN_PWM, MAX_PWM);

  
  Serial.print("Outputting PWM: "); Serial.println(write_val);

  ESC.writeMicroseconds(pwm_out);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
