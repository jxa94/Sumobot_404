// MotorTest.ino
// Comprehensive motor test for your Sumo Bot using BTS7960 motor drivers
#include <Arduino.h>

// BTS7960 Motor control pins
// Motor 1
#define M1_RPWM 11  // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M1_LPWM 10  // Left/Backward PWM pin - connect to LPWM on BTS7960
#define M1_R_EN 9   // Right/Forward enable pin - connect to R_EN on BTS7960
#define M1_L_EN 8   // Left/Backward enable pin - connect to L_EN on BTS7960

// Motor 2
#define M2_RPWM 6   // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M2_LPWM 5   // Left/Backward PWM pin - connect to LPWM on BTS7960
#define M2_R_EN 4   // Right/Forward enable pin - connect to R_EN on BTS7960
#define M2_L_EN 3   // Left/Backward enable pin - connect to L_EN on BTS7960

// Test settings
const int TEST_SPEEDS[] = {50, 100, 150, 200, 240}; 
const int NUM_SPEEDS = 5;
const int TEST_DURATION = 2000; // ms per speed level
const int PAUSE_DURATION = 1000; // ms between tests

// Function declarations
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
void turnLeft(int leftSpeed, int rightSpeed);
void turnRight(int leftSpeed, int rightSpeed);
void stopMovement();
void testMotorSequence(const char* testName, void (*motorFunction)(int, int));

void setup() {
  Serial.begin(9600);
  
  // Motor 1 pins
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M1_R_EN, OUTPUT);
  pinMode(M1_L_EN, OUTPUT);
  
  // Motor 2 pins
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M2_R_EN, OUTPUT);
  pinMode(M2_L_EN, OUTPUT);
  
  // Enable the BTS7960 drivers
  digitalWrite(M1_R_EN, HIGH);
  digitalWrite(M1_L_EN, HIGH);
  digitalWrite(M2_R_EN, HIGH);
  digitalWrite(M2_L_EN, HIGH);
  
  // Ensure motors are stopped
  stopMovement();
  
  Serial.println(F("===== Sumo Bot Motor Test - BTS7960 ====="));
  Serial.println(F("Testing individual motors and movement functions"));
  delay(2000);
}

void loop() {
  // Test individual motors first
  testSingleMotor("LEFT MOTOR FORWARD", M1_RPWM, M1_LPWM, true);
  testSingleMotor("LEFT MOTOR BACKWARD", M1_LPWM, M1_RPWM, false);
  
  testSingleMotor("RIGHT MOTOR FORWARD", M2_RPWM, M2_LPWM, true);
  testSingleMotor("RIGHT MOTOR BACKWARD", M2_LPWM, M2_RPWM, false);
  
  // Test combined movement functions
  testMotorSequence("BOTH MOTORS FORWARD", moveForward);
  testMotorSequence("BOTH MOTORS BACKWARD", moveBackward);
  testMotorSequence("TURN LEFT", turnLeft);
  testMotorSequence("TURN RIGHT", turnRight);
  
  // Test special maneuvers
  Serial.println(F("\n--- Testing special maneuvers ---"));
  
  // Quick direction reversal
  Serial.println(F("DIRECTION REVERSAL TEST"));
  moveForward(200, 200);
  delay(2000);
  moveBackward(200, 200);
  delay(2000);
  stopMovement();
  delay(PAUSE_DURATION);
  
  // Spin in place
  Serial.println(F("SPIN TEST - CLOCKWISE"));
  turnRight(200, 200);
  delay(3000);
  stopMovement();
  delay(PAUSE_DURATION);
  
  Serial.println(F("SPIN TEST - COUNTER-CLOCKWISE"));
  turnLeft(200, 200);
  delay(3000);
  stopMovement();
  delay(PAUSE_DURATION);
  
  // Sharp turn
  Serial.println(F("SHARP TURN TEST"));
  moveForward(50, 200);
  delay(3000);
  moveForward(200, 50);
  delay(3000);
  stopMovement();
  delay(PAUSE_DURATION);
  
  Serial.println(F("\n===== Motor Test Complete ====="));
  Serial.println(F("Restarting test sequence in 5 seconds..."));
  delay(5000);
}

// Function to test a single motor at different speeds
// For BTS7960: activePwm = pin to send PWM to, inactivePwm = pin to set to 0
// isForward = true for forward movement, false for backward (just for display)
void testSingleMotor(const char* testName, int activePwm, int inactivePwm, bool isForward) {
  Serial.print(F("\n--- Testing "));
  Serial.print(testName);
  Serial.println(F(" ---"));
  
  for (int i = 0; i < NUM_SPEEDS; i++) {
    int speed = TEST_SPEEDS[i];
    Serial.print(F("Speed: "));
    Serial.println(speed);
    
    analogWrite(activePwm, speed);
    analogWrite(inactivePwm, 0);  // Ensure the other direction is off
    
    delay(TEST_DURATION);
  }
  
  analogWrite(activePwm, 0);  // Stop the motor
  Serial.println(F("Test complete, motor stopped."));
  delay(PAUSE_DURATION);
}

// Function to test movement functions at different speeds
void testMotorSequence(const char* testName, void (*motorFunction)(int, int)) {
  Serial.print(F("\n--- Testing "));
  Serial.print(testName);
  Serial.println(F(" ---"));
  
  for (int i = 0; i < NUM_SPEEDS; i++) {
    int speed = TEST_SPEEDS[i];
    Serial.print(F("Speed: "));
    Serial.println(speed);
    
    motorFunction(speed, speed);
    delay(TEST_DURATION);
  }
  
  stopMovement();
  Serial.println(F("Test complete, motors stopped."));
  delay(PAUSE_DURATION);
}

// Movement Functions adapted for BTS7960
void moveForward(int leftSpeed, int rightSpeed) {
  // Left motor forward
  analogWrite(M1_RPWM, leftSpeed);
  analogWrite(M1_LPWM, 0);
  
  // Right motor forward
  analogWrite(M2_RPWM, rightSpeed);
  analogWrite(M2_LPWM, 0);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  // Left motor backward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  // Right motor backward
  analogWrite(M2_LPWM, rightSpeed);
  analogWrite(M2_RPWM, 0);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  // Left motor backward, Right motor forward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  analogWrite(M2_RPWM, rightSpeed);
  analogWrite(M2_LPWM, 0);
}

void turnRight(int leftSpeed, int rightSpeed) {
  // Left motor forward, Right motor backward
  analogWrite(M1_RPWM, leftSpeed);
  analogWrite(M1_LPWM, 0);
  
  analogWrite(M2_LPWM, rightSpeed);
  analogWrite(M2_RPWM, 0);
}

void stopMovement() {
  // Stop all motors
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, 0);
  analogWrite(M2_LPWM, 0);
}