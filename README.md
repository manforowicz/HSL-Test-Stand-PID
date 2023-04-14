# huskysat-arduino

- motor-spin: Spins a brushless motor to confirm it's working.
- PID: Uses motor to stabilize test stand using BNO055 IMU to determine orientation.
- magnetorquer-demo: Oscillates magnetorquer back and forth.

## Resources

### BNO055 IMU

- [sensors_event_t](https://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html)

- [code example](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code)

- [adafruit_sensor](https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work)

### PID

- [PID Tutorial](https://janismac.github.io/ControlChallenges/)

### Magnetorquers

- [Marcin's Video](https://youtu.be/cGJYCe6mGR0)

- [Marcin's Magnetorquer Designer](https://github.com/manforowicz/Magnetorquer-Calc)

### ESC

Good bidirectional ESCs don't need callibration, other than sending a 1500 microsecond PWM for a few seconds at the beginning.
