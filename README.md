# Marcin-Test-Stand-PID
Simple Arduino program that uses a brushless motor and BN0O55 IMU to control orientation.

BNO055 IMU Resources:
[sensors_event_t](https://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html)
[code example](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code)
[adafruit_sensor](https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work)

PID Resources:
[PID Tutorial](https://janismac.github.io/ControlChallenges/)

ESC Notes:
Good bidirectional ESCs don't need callibration, other than sending a 1500 microsecond PWM for a few seconds at the beginning.
