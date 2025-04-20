#include <Arduino.h>

// Motor pins: Digital output
#define M1L 11
#define M1R 10
#define M2L 5
#define M2R 6

// Constants for motor speed
const int DEFAULT_SPEED = 200;
const int MIN_SPEED = 0;
const int MAX_SPEED = 255;

int motorChoice = 0;
int motorSpeed = DEFAULT_SPEED;
bool isRunning = false;
unsigned long actionStartTime = 0;
unsigned long actionDuration = 2000; // Default 2-second duration for movements

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Motor Test Program");
  Serial.println("-------------------------");
  Serial.println("Enter a number to select a motor action:");
  Serial.println("1: Move Forward");
  Serial.println("2: Move Backward");
  Serial.println("3: Turn Left");
  Serial.println("4: Turn Right");
  Serial.println("5: Turn on the spot(Left)");
  Serial.println("6: Turn on the spot(Right)");
  Serial.println("7: Snake Movement");
  Serial.println("8: Stop");
  Serial.println("9: Set Motor Speed (0-255)");
  Serial.println("10: Set Action Duration (ms)");
  
  // Motor pins
  pinMode(M2R, OUTPUT);
  pinMode(M2L, OUTPUT);
  pinMode(M1L, OUTPUT);
  pinMode(M1R, OUTPUT);
  
  // Ensure motors are stopped on startup
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
}

void moveForward(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  Serial.println("Moving Forward");
}

void moveBackward(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
  Serial.println("Moving Backward");
}

void turnLeft(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2L, LOW);
  Serial.println("Turning Left");
}

void turnRight(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2R, LOW);
  Serial.println("Turning Right");
}
void turnOnSpotLeft(int leftSpeed, int rightSpeed) {
  analogWrite(M1L, leftSpeed);
  analogWrite(M2R, rightSpeed);
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  Serial.println("Turning on the spot (Left)");
}
void turnOnSpotRight(int leftSpeed, int rightSpeed) {
  analogWrite(M1R, leftSpeed);
  analogWrite(M2L, rightSpeed);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
  Serial.println("Turning on the spot (Right)");
}

void snakeMovement() {
  static bool direction = true;
  static unsigned long lastChangeTime = millis();
  unsigned long currentTime = millis();

  if (currentTime - lastChangeTime >= 500) {
    direction = !direction;
    lastChangeTime = currentTime;
  }

  if (direction) {
    moveForward(255, 150);
    Serial.println("Snake Movement (Right)");
  } else {
    moveForward(150, 255);
    Serial.println("Snake Movement (Left)");
  }
}

void stopMovement() {
  digitalWrite(M1R, LOW);
  digitalWrite(M2L, LOW);
  digitalWrite(M1L, LOW);
  digitalWrite(M2R, LOW);
  Serial.println("Stopped Movement");
  isRunning = false;
}

void loop() {
  // Check if serial data is available
  if (Serial.available() > 0) {
    // Read the incoming byte
    int input = Serial.parseInt();
    
    // Consume any remaining characters in the buffer
    while(Serial.available() > 0) {
      Serial.read();
    }
    
    // If input is 7, we're setting motor speed
    if (input == 7) {
      Serial.println("Enter motor speed (0-255):");
      while (Serial.available() == 0) {
        // Wait for input
      }
      int newSpeed = Serial.parseInt();
      if (newSpeed >= MIN_SPEED && newSpeed <= MAX_SPEED) {
        motorSpeed = newSpeed;
        Serial.print("Motor speed set to: ");
        Serial.println(motorSpeed);
      } else {
        Serial.println("Invalid speed. Using default.");
        motorSpeed = DEFAULT_SPEED;
      }
    }
    // If input is 8, we're setting action duration
    else if (input == 8) {
      Serial.println("Enter action duration in milliseconds:");
      while (Serial.available() == 0) {
        // Wait for input
      }
      actionDuration = Serial.parseInt();
      Serial.print("Action duration set to: ");
      Serial.print(actionDuration);
      Serial.println(" ms");
    }
    else {
      motorChoice = input;
      if (motorChoice >= 1 && motorChoice <= 5) {
        isRunning = true;
        actionStartTime = millis();
      }
    }
  }

  // Run the selected motor action
  if (isRunning) {
    switch (motorChoice) {
      case 1:
        moveForward(motorSpeed, motorSpeed);
        break;
      case 2:
        moveBackward(motorSpeed, motorSpeed);
        break;
      case 3:
        turnLeft(motorSpeed, motorSpeed);
        break;
      case 4:
        turnRight(motorSpeed, motorSpeed);
        break;
      case 5:
        turnOnSpotLeft(motorSpeed, motorSpeed);
        break;
      case 6:
        turnOnSpotRight(motorSpeed, motorSpeed);
        break;
      case 7:
        snakeMovement();
        break;
      case 8:
      default:
        stopMovement();
        break;
    }
    
    // Stop after the specified duration
    if (millis() - actionStartTime >= actionDuration) {
      stopMovement();
    }
  }
  else if (motorChoice == 6) {
    stopMovement();
    motorChoice = 0;
  }
}