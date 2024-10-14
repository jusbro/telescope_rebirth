//This program connects an Arduino Serial Terminal (Through the IDE) to an analog DC motor with quadrature encoder.
//The user specifies a desired RPM of the motor through the Serial Terminal
//The Arduino sends the request as a PWM to a Digital to Analog Converter (DAC) for conversion which drives the motor
//The encoder reads the "speed" of the motor and reports it back to the Arduino
//The arduino uses a Proportional Integral Derivative (PID) Calculator to determine the "fix" for the motor speed and adjusts as appropriate

// Pin definitions
const int pwmPin = 9;           // PWM pin to control motor speed
const int motorPinA = 7;        // Encoder channel A pin
const int motorPinB = 8;        // Encoder channel B pin
const int stbyPin = 10;         // Standby pin for motor driver

const int encoderPinA = 2;
const int encoderPinB = 3;

volatile long encoderTicks = 0; // Variable to store encoder ticks
int previousEncoderTicks = 0;   // For calculating speed
unsigned long lastTime = 0;     // For timing speed calculations
const int interval = 100;      // Time interval (in ms) for calculating speed

// Motor control and speed variables
int targetRPM = 0;     // Target RPM input
int pwmValue = 0;      // Mapped PWM value (0-255)
float currentRPM = 0;  // Current motor speed in RPM
const int maxRPM = 6500;    // Maximum RPM
const int minRPM = 25;     // Minimum RPM that motor can run without stalling

// PID controller variables
float kp = 0.1;  // Proportional gain
float ki = 0.05; // Integral gain
float kd = 0.01; // Derivative gain

float rpmError = 0;          // Current error
float previousRpmError = 0;  // Previous error
float errorSum = 0;          // Accumulated error for integral term
float errorRate = 0;         // Rate of change of error for derivative term

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set up PWM pin as output
  pinMode(pwmPin, OUTPUT);
  
  // Set up encoder pins as inputs with interrupts
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  digitalWrite(motorPinA, HIGH);
  digitalWrite(motorPinB, LOW);

}

void loop() {
  // Check if data is available from the serial terminal
  if (Serial.available() > 1) {
    // Read target RPM input
    targetRPM = Serial.parseInt();
    targetRPM = constrain(targetRPM, minRPM, maxRPM); // Limit input to valid RPM range
    
    // Print confirmation
    Serial.print("Target RPM set to: ");
    Serial.println(targetRPM);
  }

  // Calculate RPM every second
  if (millis() - lastTime >= interval) {
    // Calculate ticks difference
    int tickDiff = encoderTicks - previousEncoderTicks;
    previousEncoderTicks = encoderTicks;

    // Calculate current RPM: (ticks per second / ticks per revolution) * 60
    currentRPM = (tickDiff / 256.0) * 60.0;
    
    // Print current motor RPM
    Serial.print("Current RPM: ");
    Serial.println(currentRPM);

    // PID control calculation
    rpmError = targetRPM - currentRPM;       // Proportional term
    errorSum += rpmError;                    // Integral term
    errorRate = rpmError - previousRpmError; // Derivative term
    previousRpmError = rpmError;

    // Calculate the new PWM value using PID
    float pidOutput = (kp * rpmError) + (ki * errorSum) + (kd * errorRate);
    pwmValue += pidOutput;

    // Constrain PWM value to valid range (0-255)
    pwmValue = constrain(pwmValue, 0, 255);
    
    // Set the motor speed via PWM
    analogWrite(pwmPin, pwmValue);

    // Update the last time
    lastTime = millis();
  }
}

// Interrupt Service Routine (ISR) for encoder
void encoderISR() {
  // Read the current state of channel A and B
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);
  
  // Determine direction and increment/decrement encoder ticks
  if (stateA == stateB) {
    encoderTicks++; // Clockwise
  } else {
    encoderTicks--; // Counterclockwise
  }
}
