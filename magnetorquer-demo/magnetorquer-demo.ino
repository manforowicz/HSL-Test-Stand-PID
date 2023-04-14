const int OUT_1 = 3;
const int OUT_2 = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(OUT_1, OUTPUT);
  pinMode(OUT_2, OUTPUT);
}

/**
 * Sends a pwm to the magnetorquer.
 *
 * 0: Cuts off the coil.
 * 255:  Full Out 1
 * -255: Full Out 2
 */
void send(int power) {
  analogWrite(OUT_1, constrain(power, 0, 255));
  analogWrite(OUT_2, constrain(-power, 0, 255));
}

/**
 * Shorts the two ends of the coils,
 * isolating them from outside power.
 *
 * May apply a dampening effect?
 *
 */
void brake() {
  digitalWrite(OUT_1, HIGH);
  digitalWrite(OUT_2, HIGH);
}

void power_with_friction(int seconds, int power) {
  for (int i = 0; i < seconds; i++) {
    send(power);
    delay(800);
    brake();
    delay(200);
  }
}

void loop() {
  power_with_friction(40, 255);
  power_with_friction(40, -255);
}
