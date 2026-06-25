// ===================== PIN SETUP =====================
const int dirPin = 5;
const int pwmPin = 6;

const int encA = A0;
const int encB = A1;

// ===================== ENCODER =====================
volatile long encoderTicks = 0;

// ===================== TIMING =====================
unsigned long lastTime = 0;
const int interval = 100; // ms

// ===================== PID =====================
float kp = 0.5;
float ki = 0.1;
float kd = 0.0;

// Target velocity in counts per interval (NOT RPM yet)
float targetRate = 50;   // YOU WILL TUNE THIS

float prevError = 0;
float integral = 0;

// motor output
float pwm = 0;

// ===================== SETUP =====================
void setup() {
  Serial.begin(9600);

  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  pinMode(encA, INPUT);
  pinMode(encB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encA), encoderISR, CHANGE);

  digitalWrite(dirPin, LOW); // fixed direction for RA tracking
}

// ===================== LOOP =====================
void loop() {

  if (millis() - lastTime >= interval) {

    noInterrupts();
    long ticks = encoderTicks;
    encoderTicks = 0;
    interrupts();

    // ACTUAL velocity (counts per interval)
    float actualRate = ticks;

    // ERROR
    float error = targetRate - actualRate;

    // INTEGRAL
    integral += error;

    // DERIVATIVE
    float derivative = error - prevError;

    prevError = error;

    // PID OUTPUT
    float output = kp * error + ki * integral + kd * derivative;

    pwm += output;

    // clamp
    pwm = constrain(pwm, 0, 255);

    analogWrite(pwmPin, (int)pwm);

    Serial.print("Target:");
    Serial.print(targetRate);
    Serial.print(" Actual:");
    Serial.print(actualRate);
    Serial.print(" PWM:");
    Serial.println(pwm);

    lastTime = millis();
  }
}

// ===================== ISR =====================
void encoderISR() {
  int a = digitalRead(encA);
  int b = digitalRead(encB);

  if (a == b) encoderTicks++;
  else encoderTicks--;
}
