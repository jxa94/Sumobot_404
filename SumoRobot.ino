// Sumo robot for JSUMO competition
#include <Arduino.h>

// === Pin/channel assignments ===
// ADC channels for IR sensors
#define IR_LEFT_CH    A0   // A0
#define IR_MID_CH     A1   // A1
#define IR_RIGHT_CH   A2   // A2
// Digital pins (direct port)
// Back ultrasonic
#define ULTRA_TRIG_PIN A3 // PC3
#define ULTRA_ECHO_PIN A4 // PC4
// Strategy & start switch
#define STRATEGY_SELECT_PIN A5 // PC5
#define JSUMO_SWITCH_PIN    4  // PD4
// Line sensors (white=HIGH, black=LOW)
#define LINE_FL_PIN 2  // PD2
#define LINE_FR_PIN 3  // PD3
#define LINE_BL_PIN 12 // PB4
#define LINE_BR_PIN 13 // PB5
// Bump sensors
#define BUMP_LEFT_PIN  7  // PD7
#define BUMP_RIGHT_PIN 8  // PB0
// Motor pins
#define M1L_PIN 11 // PB3 (PWM & Dir)
#define M1R_PIN 10 // PB2 (Dir only)
#define M2L_PIN 5  // PD5 (Dir only)
#define M2R_PIN 6  // PD6 (PWM & Dir)

// === Thresholds ===
#define IR_DETECT_THRESHOLD    400  // ADC < 400 → object within ~30 cm
#define ULTRA_DETECT_THRESHOLD 20   // cm, back-ultrasonic

// === State machine ===
#define STATE_IDLE            0
#define STATE_SEEK            1
#define STATE_ATTACK          2
#define STATE_RETREAT         3
#define STATE_LINE_RECOVERY   4

// === Strategy selector ===
#define STRAT_DEFAULT   0
#define STRAT_RUNAWAY   1

// === Motor direction macros ===
#define SET(reg, bit)    ((reg) |=  _BV(bit))
#define CLR(reg, bit)    ((reg) &= ~_BV(bit))
// M1L_PIN = PB3, M1R_PIN = PB2
#define M1L_HIGH() SET(PORTB, PORTB3)
#define M1L_LOW()  CLR(PORTB, PORTB3)
#define M1R_HIGH() SET(PORTB, PORTB2)
#define M1R_LOW()  CLR(PORTB, PORTB2)
// M2L_PIN = PD5, M2R_PIN = PD6
#define M2L_HIGH() SET(PORTD, PORTD5)
#define M2L_LOW()  CLR(PORTD, PORTD5)
#define M2R_HIGH() SET(PORTD, PORTD6)
#define M2R_LOW()  CLR(PORTD, PORTD6)

// === Fast ADC read macro ===
#define fastADCRead(ch) ({          \
  ADMUX = _BV(REFS0) | ((ch) & 0x07); \
  ADCSRA |= _BV(ADSC);               \
  while (ADCSRA & _BV(ADSC));        \
  ADC;                                \
})

// === Globals ===
uint8_t currentState    = STATE_IDLE;
uint8_t currentStrategy = STRAT_DEFAULT;
unsigned long lastStateTime = 0, startWaitTime = 0;
bool waitingForStart = false;

// Sensor flags
bool frontDetected = false, leftDetected = false, rightDetected = false;
bool lineFL = false, lineFR = false, lineBL = false, lineBR = false;

// Raw distances
uint16_t irLeftDist, irMidDist, irRightDist, ultraDist;

// === Function prototypes ===
void stopMovement();
void moveForward(uint8_t L, uint8_t R);
void moveBackward(uint8_t L, uint8_t R);
void turnLeft(uint8_t L, uint8_t R);
void turnRight(uint8_t L, uint8_t R);
void measureUltrasonic();
void updateSensorStates();
bool lineDetected();
void executeDefaultStrategy();
void executeRunawayStrategy();

void setup() {
  // — ADC setup —
  ADCSRA = _BV(ADEN) | _BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);  // enable, prescaler=128
  DIDR0 = _BV(ADC0D)|_BV(ADC1D)|_BV(ADC2D);               // disable digital on A0–A2

  // — Motor pins as outputs —
  DDRB |= _BV(DDB3)|_BV(DDB2); // M1L, M1R
  DDRD |= _BV(DDD5)|_BV(DDD6); // M2L, M2R

  // — Sensor pins as inputs, no pull-ups —
  DDRD &= ~(_BV(DDD2)|_BV(DDD3)|_BV(DDD4)|_BV(DDD7));
  DDRB &= ~(_BV(DDB4)|_BV(DDB5)|_BV(DDB0));
  DDRC &= ~(_BV(DDC3)|_BV(DDC4)|_BV(DDC5));
  PORTD &= ~(_BV(PORTD2)|_BV(PORTD3)|_BV(PORTD4)|_BV(PORTD7));
  PORTB &= ~(_BV(PORTB4)|_BV(PORTB5)|_BV(PORTB0));
  PORTC &= ~(_BV(PORTC3)|_BV(PORTC4)|_BV(PORTC5));

  // — Initial motor stop —
  stopMovement();

  // — Initialize timing —
  lastStateTime = micros();
}

void loop() {
  // 1) Read sensors
  updateSensorStates();

  // 2) Strategy select
  currentStrategy = ( (PINC & _BV(PORTC5)) == 0 ) ? STRAT_DEFAULT : STRAT_RUNAWAY;
  // (A5 pulled low → default; high → runaway)

  // 3) Execute
  if (currentStrategy == STRAT_DEFAULT) {
    executeDefaultStrategy();
  } else {
    executeRunawayStrategy();
  }
}

// ——— SENSOR & STATE HELPERS ———

void measureUltrasonic() {
  // TRIG low→high pulse
  CLR(PORTC, PORTC3);
  delayMicroseconds(2);
  SET(PORTC, PORTC3);
  delayMicroseconds(10);
  CLR(PORTC, PORTC3);

  unsigned long t0 = micros();
  // wait for echo high
  while ((PINC & _BV(PORTC4)) == 0 && micros() - t0 < 1000UL);
  unsigned long t1 = micros();
  // wait for echo low
  while ((PINC & _BV(PORTC4)) && micros() - t1 < 30000UL);
  ultraDist = (micros() - t1) / 58; // cm
}

void updateSensorStates() {
  // IR distances
  irLeftDist  = fastADCRead(IR_LEFT_CH);
  irMidDist   = fastADCRead(IR_MID_CH);
  irRightDist = fastADCRead(IR_RIGHT_CH);

  // Ultrasonic
  measureUltrasonic();

  // Opponent detection
  frontDetected = (irMidDist  < IR_DETECT_THRESHOLD);
  leftDetected  = (irLeftDist < IR_DETECT_THRESHOLD);
  rightDetected = (irRightDist< IR_DETECT_THRESHOLD);

  // Bump switches (or back-ultra could also set a rear flag)
  leftDetected  |= (PIND & _BV(PIND7));
  rightDetected |= (PINB & _BV(PINB0));

  // Line sensors (white=HIGH, we want detect white edge → recover)
  lineFL = (PIND & _BV(PIND2));
  lineFR = (PIND & _BV(PIND3));
  lineBL = (PINB & _BV(PINB4));
  lineBR = (PINB & _BV(PINB5));
}

bool lineDetected() {
  return lineFL || lineFR || lineBL || lineBR;
}

// ——— MOTOR PRIMITIVES ———

void stopMovement() {
  // Stop PWM
  analogWrite(M1L_PIN, 0);
  analogWrite(M2R_PIN, 0);
  // All direction lines low (brake mode)
  M1L_LOW(); M1R_LOW();
  M2L_LOW(); M2R_LOW();
}

void moveForward(uint8_t L, uint8_t R) {
  // M1: forward = M1L high, M1R low, PWM on M1L
  M1L_HIGH(); M1R_LOW();
  analogWrite(M1L_PIN, L);
  // M2: forward = M2L low, M2R high, PWM on M2R
  M2L_LOW(); M2R_HIGH();
  analogWrite(M2R_PIN, R);
}

void moveBackward(uint8_t L, uint8_t R) {
  // reverse directions
  M1L_LOW(); M1R_HIGH();
  analogWrite(M1L_PIN, L);
  M2L_HIGH(); M2R_LOW();
  analogWrite(M2R_PIN, R);
}

void turnLeft(uint8_t L, uint8_t R) {
  // left wheel backward, right wheel forward
  M1L_LOW(); M1R_HIGH();
  analogWrite(M1L_PIN, L);
  M2L_LOW(); M2R_HIGH();
  analogWrite(M2R_PIN, R);
}

void turnRight(uint8_t L, uint8_t R) {
  // left wheel forward, right wheel backward
  M1L_HIGH(); M1R_LOW();
  analogWrite(M1L_PIN, L);
  M2L_HIGH(); M2R_LOW();
  analogWrite(M2R_PIN, R);
}

// ——— STRATEGY EXECUTION ———

void executeDefaultStrategy() {
  switch (currentState) {
    case STATE_IDLE:
      stopMovement();
      // Start button held?
      if (PIND & _BV(PIND4)) {
        if (!waitingForStart) {
          startWaitTime = micros();
          waitingForStart = true;
        }
        if (micros() - startWaitTime >= 3000000UL) {
          currentState = STATE_SEEK;
          lastStateTime = micros();
        }
      } else {
        waitingForStart = false;
      }
      break;

    case STATE_SEEK:
      turnRight(150,150);
      if (frontDetected||leftDetected||rightDetected) {
        currentState = STATE_ATTACK; lastStateTime = micros();
      }
      if (lineDetected()) {
        currentState = STATE_LINE_RECOVERY; lastStateTime = micros();
      }
      break;

    case STATE_ATTACK:
      if (frontDetected) moveForward(255,255);
      else if (leftDetected) moveForward(150,255);
      else if (rightDetected) moveForward(255,150);
      else { currentState = STATE_SEEK; lastStateTime = micros(); }
      if (lineDetected()) { currentState = STATE_LINE_RECOVERY; lastStateTime = micros(); }
      break;

    case STATE_LINE_RECOVERY:
      // if front edge, back off
      if (lineFL||lineFR) {
        moveBackward(200,200);
        if (micros() - lastStateTime >= 300000UL) {
          currentState = STATE_RETREAT; lastStateTime = micros();
        }
      }
      // if back edge, drive forward
      else if (lineBL||lineBR) {
        moveForward(200,200);
        if (micros() - lastStateTime >= 300000UL) {
          currentState = STATE_RETREAT; lastStateTime = micros();
        }
      }
      else {
        currentState = STATE_SEEK; lastStateTime = micros();
      }
      break;

    case STATE_RETREAT:
      turnRight(255,255);
      if (micros() - lastStateTime >= 400000UL) {
        currentState = STATE_SEEK; lastStateTime = micros();
      }
      break;
  }
}

void executeRunawayStrategy() {
  switch (currentState) {
    case STATE_IDLE:
      stopMovement();
      if (PIND & _BV(PIND4)) {
        if (!waitingForStart) {
          startWaitTime = micros();
          waitingForStart = true;
        }
        if (micros() - startWaitTime >= 3000000UL) {
          currentState = STATE_SEEK; lastStateTime = micros();
        }
      } else waitingForStart = false;
      break;

    case STATE_SEEK:
      moveForward(150,100); // weave
      if (frontDetected||leftDetected||rightDetected) {
        currentState = STATE_RETREAT; lastStateTime = micros();
      }
      if (lineDetected()) {
        currentState = STATE_LINE_RECOVERY; lastStateTime = micros();
      }
      break;

    case STATE_RETREAT:
      if (frontDetected) moveBackward(255,255);
      else if (leftDetected) turnRight(255,255);
      else if (rightDetected) turnLeft(255,255);
      else if (micros() - lastStateTime >= 500000UL) {
        currentState = STATE_SEEK; lastStateTime = micros();
      }
      if (lineDetected()) {
        currentState = STATE_LINE_RECOVERY; lastStateTime = micros();
      }
      break;

    case STATE_LINE_RECOVERY:
      // same as default
      if (lineFL||lineFR) {
        moveBackward(200,200);
        if (micros() - lastStateTime >= 300000UL) {
          currentState = STATE_ATTACK; lastStateTime = micros();
        }
      }
      else if (lineBL||lineBR) {
        moveForward(200,200);
        if (micros() - lastStateTime >= 300000UL) {
          currentState = STATE_ATTACK; lastStateTime = micros();
        }
      }
      else {
        currentState = STATE_SEEK; lastStateTime = micros();
      }
      break;

    case STATE_ATTACK:
      turnRight(200,200);
      if (micros() - lastStateTime >= 500000UL) {
        currentState = STATE_SEEK; lastStateTime = micros();
      }
      break;
  }
}
