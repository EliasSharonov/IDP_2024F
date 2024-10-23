// Declare pins here
#define PWMOUT_PIN 10  // PB2
#define VINREG_PIN A0

// Declare other constants here
#define FET_PERIOD 10 //microseconds
#define DIVIDER_CONSTANT 11 // Approximately
#define IDEAL_DUTY_CYCLE 0.53
#define DUTY_CYCLE_CONSTANT 0.01
#define IDEAL_INPUT_VOLTAGE 10.7

// Variables
float outVoltage;
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
#define P 0.001
#define I 0.0
#define D 0

void setup() {
  pinMode(PWMOUT_PIN, OUTPUT);
  pinMode(4, OUTPUT);
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
}

void loop() {
  unsigned long currentTime = millis();

  updatePWM();

  if (currentTime - lastVoltageReadTime >= 10) {
    readVoltage();
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

void readVoltage() {
  outVoltage = ((float)analogRead(VINREG_PIN) * DIVIDER_CONSTANT) / 203.0;
}

void adjustPID() {
  unsigned long currentMicros = micros();
  dt = (currentMicros - lastTime) / 1000000.0;
  lastTime = currentMicros;

  error = IDEAL_INPUT_VOLTAGE - outVoltage;
  integral += error * dt;
  derivative = (error - previousError) / dt;

  dutyCycle = 0.53 + P * error + I * integral;
  // 0.1*((P * error) + (I * integral) + (D * derivative));

  if (dutyCycle > 1) dutyCycle = 1;
  if (dutyCycle < 0) dutyCycle = 0;

  previousError = error;
}
