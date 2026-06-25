const int pwmPin = 6;

void setup() {
  pinMode(pwmPin, OUTPUT);
  Serial.begin(115200);

  analogWriteFrequency(pwmPin, 20000); // start at 20 kHz
}

void loop() {

  if (Serial.available()) {

    int val = Serial.parseInt();

    val = constrain(val, 0, 255);

    analogWrite(pwmPin, val);

    Serial.print("PWM set to: ");
    Serial.println(val);
  }
}
