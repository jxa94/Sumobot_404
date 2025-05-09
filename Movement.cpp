#include "Movement.h"

// Movement Functions for BTS7960 - MODIFIED for parallel motors
void moveForward(int leftSpeed, int rightSpeed) {
  // Left motor forward (one direction)
  analogWrite(M1_RPWM, leftSpeed);
  analogWrite(M1_LPWM, 0);
  
  // Right motor forward (opposite direction due to parallel setup)
  analogWrite(M2_LPWM, rightSpeed);
  analogWrite(M2_RPWM, 0);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  // Left motor backward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  // Right motor backward (opposite direction due to parallel setup)
  analogWrite(M2_RPWM, rightSpeed);
  analogWrite(M2_LPWM, 0);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  // Left motor backward, Right motor backward
  analogWrite(M1_LPWM, leftSpeed);
  analogWrite(M1_RPWM, 0);
  
  analogWrite(M2_RPWM, rightSpeed);
  analogWrite(M2_LPWM, 0);
}

void turnRight(int leftSpeed, int rightSpeed) {
  // Left motor forward, Right motor forward
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