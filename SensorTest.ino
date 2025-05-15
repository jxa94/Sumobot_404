// SensorTest.ino
// Comprehensive sensor testing for Sumo Bot (Line sensors removed)
#include <Arduino.h>

// BTS7960 Motor control pins
// Motor right
#define M2R 10  // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M2L 11  // Left/Backward PWM pin - connect to LPWM on BTS7960

// Motor left
#define M1R 5   // Right/Forward PWM pin - connect to RPWM on BTS7960
#define M1L 6   // Left/Backward PWM pin - connect to LPWM on BTS7960

// Starter switch: Digital input
#define JSUMO_SWITCH 2

// Bump sensors
#define BUMP_LEFT 12
#define BUMP_RIGHT 4

// IR reflectance sensors: Analog input
#define IR_REFLECT_LEFT A0
#define IR_REFLECT_CENTER A1
#define IR_REFLECT_RIGHT A2

// Ultrasonic sensor: Analog input
#define ULTRASONIC_TRIG A3
#define ULTRASONIC_ECHO A4

// Constants
const int BUMP_LEFT_BIT = 0;
const int BUMP_RIGHT_BIT = 1;

// IR sensor constants
const int IR_MAX_DISTANCE = 150; // cm - maximum reliable distance
const int IR_MIN_DISTANCE = 20;  // cm - minimum reliable distance

// Function declarations
int getIRDistance(int sensorPin);
long getUltrasonicDistance();
void updateSensorStates();
void displaySensorReadings();

// Global variables to store sensor states
int bumpSensorState = 0;
int leftIRValue = 0;
int centerIRValue = 0;
int rightIRValue = 0;
int leftIRDistance = 0;
int centerIRDistance = 0;
int rightIRDistance = 0;
long ultrasonicDistance = 0;

void setup() {
  Serial.begin(9600);

  // IR sensors
  pinMode(IR_REFLECT_LEFT, INPUT);
  pinMode(IR_REFLECT_CENTER, INPUT);
  pinMode(IR_REFLECT_RIGHT, INPUT);

  // Ultrasonic sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // Bump sensors
  pinMode(BUMP_LEFT, INPUT_PULLUP);
  pinMode(BUMP_RIGHT, INPUT_PULLUP);
  
  // Starter switch (if testing)
  pinMode(JSUMO_SWITCH, INPUT);

  Serial.println(F("===== Sumo Bot Sensor Test ====="));
  Serial.println(F("Monitoring sensors: IR, Ultrasonic, and Bump"));
  delay(1000);
}

void loop() {
  // Update all sensor readings
  updateSensorStates();
  
  // Display current readings
  displaySensorReadings();
  
  // Add a brief delay between readings
  delay(500);
}

// Update sensor states
void updateSensorStates() {
  // Bump sensors (active LOW with pullup)
  bumpSensorState = (!digitalRead(BUMP_LEFT) << BUMP_LEFT_BIT) |
                    (!digitalRead(BUMP_RIGHT) << BUMP_RIGHT_BIT);
  
  // Read IR sensor raw values
  leftIRValue = analogRead(IR_REFLECT_LEFT);
  centerIRValue = analogRead(IR_REFLECT_CENTER);
  rightIRValue = analogRead(IR_REFLECT_RIGHT);
  
  // Convert IR readings to distances
  leftIRDistance = getIRDistance(IR_REFLECT_LEFT);
  centerIRDistance = getIRDistance(IR_REFLECT_CENTER);
  rightIRDistance = getIRDistance(IR_REFLECT_RIGHT);
  
  // Get ultrasonic distance
  ultrasonicDistance = getUltrasonicDistance();
}

// IR Distance conversion (using the same formula as your driver code)
int getIRDistance(int sensorPin) {
  int reading = analogRead(sensorPin);
  
  // Convert analog reading (0-1023) to voltage (0-5V)
  float voltage = reading * (5.0 / 1023.0);
  
  // Check if voltage is too low (beyond maximum distance)
  if (voltage < 0.5) return IR_MAX_DISTANCE;
  
  // Convert voltage to distance using the inverse relationship
  // The formula is derived from the second graph showing 1/distance vs voltage
  float inverse_distance = (voltage - 0.42) / 42.5;
  float distance = 1.0 / inverse_distance;
  
  // Apply limits
  if (distance > IR_MAX_DISTANCE) return IR_MAX_DISTANCE;
  if (distance < IR_MIN_DISTANCE) return IR_MIN_DISTANCE;
  
  return (int)distance;
}

// Ultrasonic Distance measurement
long getUltrasonicDistance() {
  // Clear the trigger pin
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin high for 10 microseconds
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Read the echo pin, return the sound wave travel time in microseconds
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // Timeout after 30ms
  
  // If no signal received
  if (duration == 0) return -1;
  
  // Calculate distance in centimeters
  return duration * 0.034 / 2;
}

// Display all sensor readings in a formatted way
void displaySensorReadings() {
  // Display IR sensor data
  Serial.println(F("\n----- Sensor Readings -----"));
  
  Serial.println(F("-- IR Sensors --"));
  Serial.print(F("Left:   Raw=")); Serial.print(leftIRValue);
  Serial.print(F("\tDistance=")); Serial.print(leftIRDistance);
  Serial.println(F(" cm"));
  
  Serial.print(F("Center: Raw=")); Serial.print(centerIRValue);
  Serial.print(F("\tDistance=")); Serial.print(centerIRDistance);
  Serial.println(F(" cm"));
  
  Serial.print(F("Right:  Raw=")); Serial.print(rightIRValue);
  Serial.print(F("\tDistance=")); Serial.print(rightIRDistance);
  Serial.println(F(" cm"));
  
  // Display ultrasonic sensor data
  Serial.println(F("\n-- Ultrasonic Sensor --"));
  Serial.print(F("Distance: "));
  if (ultrasonicDistance < 0) {
    Serial.println(F("No signal received"));
  } else {
    Serial.print(ultrasonicDistance);
    Serial.println(F(" cm"));
  }
  
  // Display bump sensors
  Serial.println(F("\n-- Bump Sensors --"));
  Serial.print(F("Left: ")); Serial.println((bumpSensorState & (1 << BUMP_LEFT_BIT)) ? "PRESSED" : "NOT PRESSED");
  Serial.print(F("Right: ")); Serial.println((bumpSensorState & (1 << BUMP_RIGHT_BIT)) ? "PRESSED" : "NOT PRESSED");
  
  // Display starter switch status (if applicable)
  Serial.println(F("\n-- Starter Switch --"));
  Serial.print(F("Switch: ")); Serial.println(digitalRead(JSUMO_SWITCH) ? "ON" : "OFF");
  
  Serial.println(F("-------------------------"));
}