// EnhancedMotorTest.ino
// Comprehensive motor test for your Sumo Bot using same style as driver code
#include <Arduino.h>

// Motor pins: Digital output (matching your original driver)
#define M1L 11
#define M1R 10
#define M2L 5
#define M2R 6

// Test settings
const int TEST_SPEEDS[] = {50, 100, 150, 200, 240}; 
const int NUM_SPEEDS = 5;
const int TEST_DURATION = 2000; // ms per speed level
const int PAUSE_DURATION = 1000; // ms between tests

// Function declarations (matching your driver style)
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
void turnLeft(int leftSpeed, int rightSpeed);
void turnRight(int leftSpeed, int rightSpeed);
void stopMovement();
void testMotorSequence(const char* testName, void (*motorFunction)(int, int));

void setup() {
  Serial.begin(9600);
  
  // Motor pins
  pinMode(M1L, OUTPUT);
  pinMode(M1R, OUTPUT);
  pinMode(M2L, OUTPUT);
  pinMode(M2R, OUTPUT);
  
  // Ensure motors are stopped
  stopMovement();
  
  Serial.println(F("===== Sumo Bot Motor Test ====="));
  Serial.println(F("Testing individual motors and movement functions"));
  delay(2000);
}

void loop() {
  // Test individual motors first
  testSingleMotor("LEFT MOTOR FORWARD", M1L, M1R, LOW);
  testSingleMotor("LEFT MOTOR BACKWARD", M1L, M1R, HIGH);
  
  testSingleMotor("RIGHT MOTOR FORWARD", M2R, M2L, LOW);
  testSingleMotor("RIGHT MOTOR BACKWARD", M2R, M2L, HIGH);
  
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
void testSingleMotor(const char* testName, int pwmPin, int dirPin, int dirValue) {
  Serial.print(F("\n--- Testing "));
  Serial.print(testName);
  Serial.println(F(" ---"));
  
  digitalWrite(dirPin, dirValue);
  
  for (int i = 0; i < NUM_SPEEDS; i++) {
    int speed = TEST_SPEEDS[i];
    Serial.print(F("Speed: "));
    Serial.println(speed);
    
    analogWrite(pwmPin, speed);
    delay(TEST_DURATION);
  }
  
  analogWrite(pwmPin, 0);
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

// Movement Functions (identical to your driver code)
void moveForward(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2L, LOW);
}

void turnRight(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2R, LOW);
}

void stopMovement() {
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
}