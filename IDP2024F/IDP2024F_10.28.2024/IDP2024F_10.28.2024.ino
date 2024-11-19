// Declare pins here
#define PWMOUT_PIN 10  // PB2
#define VINREG_PIN A0
#define BATTERY_PIN A1
#define THERMISTOR_PIN A2
#define SLEEP_INDICATOR_PIN 11  // Pin to indicate sleep mode
#define VINPUT_PIN A3

// Declare other constants here
#define FET_PERIOD 10 //microseconds
#define DIVIDER_CONSTANT 11 // Approximately
#define IDEAL_DUTY_CYCLE 0.6 // Increased baseline for higher duty cycle start
#define MAX_OUT_VOLTAGE 11.5 // Maximum output voltage limit
#define THERMISTOR_SERIAL_RESISTANCE 10000.0
#define VOLTAGE_THRESHOLD 5
#define WAKE_UP_INTERVAL 30000 // (ms)

// Variables
float outVoltage;
float batteryVoltage;
int thermistorVoltage;
float sourceVoltage;
float dutyCycle = IDEAL_DUTY_CYCLE;

// PID variables
float error = 0;
float integral = 0;
float previousError = 0;
float dt = 0;
float derivative =  0;
float temperature;
volatile bool wdtWakeUp = false;
volatile uint8_t wdtInterruptCount = 0;

unsigned long lastTime = 0;
unsigned long lastVoltageReadTime = 0;
unsigned long lastPIDTime = 0;

// Temperature constants
#define TEMPERATURE_C1 1.009249522e-03
#define TEMPERATURE_C2 378405444e-04
#define TEMPERATURE_C3 2.019202697e-07

// PID constants
#define kP 0.01  // Slightly increased proportional gain
#define kI 0.05  // Increased integral gain to push the duty cycle higher
#define kD 0.0

// SETUP
void setup() {
  pinMode(PWMOUT_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(VINREG_PIN, INPUT);
  pinMode(SLEEP_INDICATOR_PIN, OUTPUT); // Initialize sleep indicator pin
  
  digitalWrite(SLEEP_INDICATOR_PIN, HIGH); // Start with HIGH to indicate active mode

  noInterrupts();

  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDP3) | (1 << WDP0); // Set timeout to 8 seconds
  WDTCSR |= (1 << WDIE);

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

// PROGRAM LOOP
void loop() {
  unsigned long currentTime = micros();

  // Sleep mode control
  sourceVoltage = ((float)analogRead(VINPUT_PIN)*DIVIDER_CONSTANT)/ 203.0;
  if ((sourceVoltage <= VOLTAGE_THRESHOLD)) {
    digitalWrite(SLEEP_INDICATOR_PIN, LOW); // Set LOW to indicate sleep mode
    Serial.println("Sleep Mode"); 
    enterSleepLoop();
  }

  digitalWrite(SLEEP_INDICATOR_PIN, HIGH); // Set HIGH to indicate active mode


  // Duty cycle output
  OCR1B = (uint16_t)(dutyCycle * ICR1);

  // Voltage reading
  if (currentTime - lastVoltageReadTime >= 10) {
    outVoltage = ((float)analogRead(VINREG_PIN) * DIVIDER_CONSTANT) / 203.0;
    batteryVoltage = ((float)analogRead(BATTERY_PIN) * DIVIDER_CONSTANT) / 203.0;
    lastVoltageReadTime = currentTime;
  }
  thermistorVoltage = analogRead(THERMISTOR_PIN);
  convertTemperature();

  // Duty cycle adjustment
  if (currentTime - lastPIDTime >= 10) {
    adjustPID();
    lastPIDTime = currentTime;
  }

  delay(500);
}

// HELPER FUNCTIONS
void convertTemperature() {
  temperature = THERMISTOR_SERIAL_RESISTANCE * (1023.0 / (float)thermistorVoltage - 1.0);
  temperature = log(temperature);
  temperature = (1.0 / (TEMPERATURE_C1 + TEMPERATURE_C2*temperature + TEMPERATURE_C3*temperature*temperature*temperature));
  temperature = temperature - 273.15;
  temperature = (temperature * 9.0) / 5.0 + 32.0; 
}

void debugInfo() {
  Serial.print("Temperature: "); 
  Serial.print(temperature);
  Serial.println(" F"); 
  Serial.print("Duty Cycle: "); 
  Serial.println(dutyCycle);
  Serial.print("Out Voltage: "); 
  Serial.print(outVoltage);
  Serial.println(" V"); 
  Serial.print("Battery voltage: "); 
  Serial.print(batteryVoltage);
  Serial.println(" V"); 
  Serial.print("Source voltage: "); 
  Serial.print(sourceVoltage);
  Serial.println(" V"); 
}

void adjustPID() {
  unsigned long currentMicros = micros();
  dt = (currentMicros - lastTime) / 1000000.0;
  lastTime = currentMicros;

  error = batteryVoltage + 1.0 - outVoltage;

  integral += error * dt;  // Accumulate integral without clamping
  derivative = (error - previousError) / dt;

  dutyCycle = IDEAL_DUTY_CYCLE + (kP * error) + (kI * integral) + (kD * derivative);

  if (dutyCycle > 1) dutyCycle = 1;
  if (dutyCycle < 0) dutyCycle = 0;

  previousError = error;
}

// SLEEP LOOP
void enterSleepLoop() {
  SMCR = (1 << SM1) | (1 << SE);
  while (true) {
    SMCR |= (1 << SE);
    do {
      __asm__ __volatile__ ( "sleep" "\n\t" :: );
    } while(0);
    SMCR &= ~(1 << SE);

    if (wdtWakeUp) {
      wdtWakeUp = false;

      wdtInterruptCount++;

      if (wdtInterruptCount >= 4) { // 8*4 = 32 seconds
        wdtInterruptCount = 0;
        sourceVoltage = ((float)analogRead(VINPUT_PIN)*DIVIDER_CONSTANT) / 203.0;
        if (sourceVoltage > VOLTAGE_THRESHOLD) {
          digitalWrite(SLEEP_INDICATOR_PIN, HIGH); // Set HIGH when waking up
          return;
        }
      }
    }
  }
}

// Interrupt control
ISR(WDT_vect) {
  Serial.println("Awake Mode"); 
  debugInfo();

  wdtWakeUp = true;
}
