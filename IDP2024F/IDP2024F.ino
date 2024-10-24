#include "timer.h"

// Declare pins here
#define PWMOUT_PIN 10  // PB2
#define VINREG_PIN A0
#define BATTERY_PIN A1

// Declare other constants here
#define FET_PERIOD 10 //microseconds
#define DIVIDER_CONSTANT 11 // Approximately
#define IDEAL_DUTY_CYCLE 0.53
#define DUTY_CYCLE_CONSTANT 0.01
#define IDEAL_INPUT_VOLTAGE 10.0

// Variables
float outVoltage;
float batteryVoltage;
float dutyCycle = IDEAL_DUTY_CYCLE;

// PID variables
float error = 0;
float integral = 0;
float previousError = 0;
float dt = 0;
float derivative =  0;

unsigned long lastTime = 0;
unsigned long lastVoltageReadTime = 0;
unsigned long lastPIDTime = 0;

// PID constants
#define kP 0.01
#define kI 0.1
#define kD 0.0

void setup() {
  pinMode(PWMOUT_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(VINREG_PIN, INPUT);

  noInterrupts();

  TCCR1A = (1 << COM1B1) | (1 << WGM11); 
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);

  ICR1 = 160;  
  OCR1B = (uint16_t)(IDEAL_DUTY_CYCLE * ICR1);  

  interrupts();

  lastTime = micros();
  lastVoltageReadTime = millis();
  lastPIDTime = millis();
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = micros();

  updatePWM();

  if (currentTime - lastVoltageReadTime >= 10) {
    readOutVoltage();
    readBatteryVoltage();
    lastVoltageReadTime = currentTime;
  }

  if (currentTime - lastPIDTime >= 10) {
    adjustPID();
    lastPIDTime = currentTime;
  }
}

void updatePWM() {
  OCR1B = (uint16_t)(dutyCycle * ICR1);
}

void readOutVoltage() {
  outVoltage = ((float)analogRead(VINREG_PIN) * DIVIDER_CONSTANT) / 203.0;
}

void readBatteryVoltage() {
  batteryVoltage = ((float)analogRead(BATTERY_PIN) * DIVIDER_CONSTANT) / 203.0;
}

void adjustPID() {
  unsigned long currentMicros = micros();
  dt = (currentMicros - lastTime) / 1000000.0;
  lastTime = currentMicros;

  error = batteryVoltage + 1.5 - outVoltage;
    Serial.println(error);
  integral += error * dt;
  derivative = (error - previousError) / dt;

  dutyCycle = 0.53 + ((kP * error) + (kI * integral) + (kD * derivative));

  if (dutyCycle > 1) dutyCycle = 1;
  if (dutyCycle < 0) dutyCycle = 0;

  previousError = error;
}
