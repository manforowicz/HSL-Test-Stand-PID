#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// MARCIN'S BARE-BONES TEST-STAND PID

//   BNO055 Connections
//   ============
//   CSCL --- A5
//   SDA ---- A4
//   VDD ---- 3.3-5V DC
//   GND ---- Common Ground

// ESC Connections
// ==============
// Dark ----- GND
// Yellow --- 9



// Delay between loops
const int BNO055_SAMPLERATE_DELAY_MS = 10;

// Units: microseconds
const float STATIONARY_PWM = 1500; // PWM where the motor doesn't spin
const float MAX_PWM = 1700;
const float MIN_PWM = 1300;

// Direction of motor spin
const bool REVERSE_MOTOR = true;

// The target angle for PID to aim at
float target_angle;

// Pretending ESC is servo
Servo ESC;
Adafruit_BNO055 bno = Adafruit_BNO055(55);




// Displays sensor reading
void printSensorEvent(sensors_event_t* event) {
  // roll/x range: (-90, 90)
  // pitch/y range: (-180, 180)
  // heading/z range: (0, 359)
  
  Serial.print("(roll/x, pitch/y, heading/z) ");
  
  Serial.print("Orientation: (");
  Serial.print(event -> orientation.x);
  Serial.print(", ");
  Serial.print(event -> orientation.y);
  Serial.print(", ");
  Serial.print(event -> orientation.z);
  Serial.print(") ");

  Serial.print("Gyro: (");
  Serial.print(event -> gyro.x);
  Serial.print(", ");
  Serial.print(event -> gyro.y);
  Serial.print(", ");
  Serial.print(event -> gyro.z);
  Serial.println(")");
}

void spinMotor(float velocity) {
  if (REVERSE_MOTOR) {
    velocity = -velocity;
  }
  float pwm_out = STATIONARY_PWM + velocity;

  // constrain pwm to range for safety
  pwm_out = constrain(pwm_out, MIN_PWM, MAX_PWM);
  
  ESC.writeMicroseconds(pwm_out);
}


void setup() {
  Serial.begin(9600);
  
  // Set up the IMU
  if(!bno.begin()) {
    Serial.println("BNO055 not detected! Quitting!");
    exit(1);
  }
  bno.setExtCrystalUse(true);

  // Initialize ESC by writting STATIONARY_PWM for 3 seconds
  ESC.attach(9);
  ESC.writeMicroseconds(STATIONARY_PWM);
  delay(3000);

  // Set the target_pos to current position
  sensors_event_t event;
  bno.getEvent(&event);
  target_angle = event.orientation.roll;

  Serial.println("Setup finished!");
}

void loop() {
  // store sensor data in "event". Documentation:
  // https://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html
  sensors_event_t event;
  bno.getEvent(&event);
  printSensorEvent(&event);

  float velocity = event.gyro.x; // gyro gives velocity
  float angle = event.orientation.x;

  float P = 1 * (target_angle - angle);
  float I = 0; // set to zero for now
  float D = -0.1 * (velocity); // set to zero for now

  spinMotor(P + I + D);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
