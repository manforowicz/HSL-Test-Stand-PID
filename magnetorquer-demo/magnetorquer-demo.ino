const int IN_1 = 3;
const int IN_2 = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  

}

void positive() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
}

void negative() {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
}

void brake() {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, HIGH);
}

void off() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
}

void loop() {
  positive();
  delay(30 * 1000);
  negative();
  delay(30 * 1000);

}
