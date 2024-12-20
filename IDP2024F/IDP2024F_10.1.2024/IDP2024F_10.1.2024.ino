// Declare pins here
#define PWMOUT_PIN 10  // PB2
#define VINREG_PIN A0
#define BATTERY_PIN A1
#define THERMISTOR_PIN A2

// Declare other constants here
#define FET_PERIOD 10 //microseconds
#define DIVIDER_CONSTANT 11 // Approximately
#define IDEAL_DUTY_CYCLE 0.6 // Increased baseline for higher duty cycle start
#define MAX_OUT_VOLTAGE 11.5 // Maximum output voltage limit

#define THERMISTOR_NOMINAL 10000.0
#define TEMPERATURE_NOMINAL 25.0
#define B_COEFFICIENT 3350.0
#define SERIES_RESISTOR 10000.0

// Variables
float outVoltage;
float batteryVoltage;
float temperature;

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
#define kP 0.03  // Slightly increased proportional gain
#define kI 0.05  // Increased integral gain to push the duty cycle higher
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

  OCR1B = (uint16_t)(dutyCycle * ICR1);

  if (currentTime - lastVoltageReadTime >= 10) {
    outVoltage = ((float)analogRead(VINREG_PIN) * DIVIDER_CONSTANT) / 203.0;
    batteryVoltage = ((float)analogRead(BATTERY_PIN) * DIVIDER_CONSTANT) / 203.0;
    lastVoltageReadTime = currentTime;
  }

  if (currentTime - lastPIDTime >= 10) {
    adjustPID();
    lastPIDTime = currentTime;
  }

    temperature = log(SERIES_RESISTOR / (1023.0 / (float)analogRead(THERMISTOR_PIN) - 1.0));
    Serial.print("Debug resistance: ");
    Serial.println(temperature);

    temperature = (1.009249522e-03 + 2.378405444e-04 * temperature + 2.019202697e-07 * temperature * temperature * temperature) - 273.15;
    // Debugging
    Serial.print("Temperature: ");
    Serial.println(temperature);


  delay(500);
}

void adjustPID() {
  unsigned long currentMicros = micros();
  dt = (currentMicros - lastTime) / 1000000.0;
  lastTime = currentMicros;

  error = batteryVoltage + 1.0 - outVoltage;

  integral += error * dt;  // Accumulate integral without clamping
  derivative = (error - previousError) / dt;

  dutyCycle = IDEAL_DUTY_CYCLE + (kP * error) + (kI * integral) + (kD * derivative);

  // Clamp the duty cycle between 0 and 1
  if (dutyCycle > 1) dutyCycle = 1;
  if (dutyCycle < 0) dutyCycle = 0;

  // Debugging information
  previousError = error;
}
