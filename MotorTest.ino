// MotorTest.ino
// Safe motor test for your Sumo Bot

#include <Arduino.h>

// Motor pins (match your wiring)
#define M1L_PIN 11  // PB3, PWM
#define M1R_PIN 10  // PB2, direction
#define M2L_PIN 5   // PD5, direction
#define M2R_PIN 6   // PD6, PWM

void setup() {
  Serial.begin(9600);
  // Configure pins
  pinMode(M1L_PIN, OUTPUT);
  pinMode(M1R_PIN, OUTPUT);
  pinMode(M2L_PIN, OUTPUT);
  pinMode(M2R_PIN, OUTPUT);

  // Ensure motors are stopped
  analogWrite(M1L_PIN, 0);
  analogWrite(M2R_PIN, 0);
  digitalWrite(M1R_PIN, LOW);
  digitalWrite(M2L_PIN, LOW);

  Serial.println("--- MotorTest Starting ---");
  delay(1000);
}

void loop() {
  // Test Motor 1 (Left)
  Serial.println("Motor 1 Forward");
  digitalWrite(M1R_PIN, LOW);
  for (int speed = 50; speed <= 150; speed += 50) {
    analogWrite(M1L_PIN, speed);
    Serial.print("Speed: "); Serial.println(speed);
    delay(2000);
  }
  analogWrite(M1L_PIN, 0);
  delay(500);

  Serial.println("Motor 1 Reverse");
  digitalWrite(M1R_PIN, HIGH);
  for (int speed = 50; speed <= 150; speed += 50) {
    analogWrite(M1L_PIN, speed);
    Serial.print("Speed: "); Serial.println(speed);
    delay(2000);
  }
  analogWrite(M1L_PIN, 0);
  delay(1000);

  // Test Motor 2 (Right)
  Serial.println("Motor 2 Forward");
  digitalWrite(M2L_PIN, LOW);
  for (int speed = 50; speed <= 150; speed += 50) {
    analogWrite(M2R_PIN, speed);
    Serial.print("Speed: "); Serial.println(speed);
    delay(2000);
  }
  analogWrite(M2R_PIN, 0);
  delay(500);

  Serial.println("Motor 2 Reverse");
  digitalWrite(M2L_PIN, HIGH);
  for (int speed = 50; speed <= 150; speed += 50) {
    analogWrite(M2R_PIN, speed);
    Serial.print("Speed: "); Serial.println(speed);
    delay(2000);
  }
  analogWrite(M2R_PIN, 0);
  delay(1000);

  Serial.println("--- MotorTest Complete ---");
  while (true) delay(1000);
}
