const int pwmPin = 6;

void setup() {
  pinMode(pwmPin, OUTPUT);
}

void loop() {

  // sweep PWM frequency
  long freqs[] = {100, 500, 1000, 5000, 10000, 20000, 25000};

  for (int i = 0; i < 7; i++) {

    analogWriteFrequency(pwmPin, freqs[i]);
    analogWrite(pwmPin, 128);   // 50% duty cycle

    delay(3000); // run each frequency for 3 seconds
  }
}
