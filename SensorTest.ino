
// ---

// SensorTest.ino
// Comprehensive sensor-check for IR, Ultrasonic, Line, and Bump

#include <Arduino.h>

// IR sensors
#define IR_LEFT_PIN   A0
#define IR_MID_PIN    A1
#define IR_RIGHT_PIN  A2
// Ultrasonic sensor
#define ULTRA_TRIG_PIN A3
#define ULTRA_ECHO_PIN A4
// Line sensors (white=HIGH)
#define LINE_FL_PIN 2
#define LINE_FR_PIN 3
#define LINE_BL_PIN 12
#define LINE_BR_PIN 13
// Bump sensors (pressed=LOW)
#define BUMP_LEFT_PIN 7
#define BUMP_RIGHT_PIN 8

void setup() {
  Serial.begin(9600);

  // Ultrasonic pins
  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);

  // Line sensors
  pinMode(LINE_FL_PIN, INPUT_PULLUP);
  pinMode(LINE_FR_PIN, INPUT_PULLUP);
  pinMode(LINE_BL_PIN, INPUT_PULLUP);
  pinMode(LINE_BR_PIN, INPUT_PULLUP);

  // Bump sensors
  pinMode(BUMP_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUMP_RIGHT_PIN, INPUT_PULLUP);

  Serial.println("--- SensorTest Starting ---");
}

long measureUltra() {
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  long duration = pulseIn(ULTRA_ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration / 58; // cm
}

void loop() {
  int irL = analogRead(IR_LEFT_PIN);
  int irM = analogRead(IR_MID_PIN);
  int irR = analogRead(IR_RIGHT_PIN);

  long uDist = measureUltra();

  bool lnFL = digitalRead(LINE_FL_PIN);
  bool lnFR = digitalRead(LINE_FR_PIN);
  bool lnBL = digitalRead(LINE_BL_PIN);
  bool lnBR = digitalRead(LINE_BR_PIN);

  bool bpL = !digitalRead(BUMP_LEFT_PIN);
  bool bpR = !digitalRead(BUMP_RIGHT_PIN);

  Serial.print("IR L="); Serial.print(irL);
  Serial.print(" M="); Serial.print(irM);
  Serial.print(" R="); Serial.print(irR);

  Serial.print(" | Ultra="); Serial.print(uDist);

  Serial.print(" | Line FL="); Serial.print(lnFL);
  Serial.print(" FR="); Serial.print(lnFR);
  Serial.print(" BL="); Serial.print(lnBL);
  Serial.print(" BR="); Serial.print(lnBR);

  Serial.print(" | Bump L="); Serial.print(bpL);
  Serial.print(" R="); Serial.println(bpR);

  delay(500);
}
