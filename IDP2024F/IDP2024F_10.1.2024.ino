// Declare pins here
#define PWMOUT_PIN 10
#define VINREG_PIN 9

// Declare other constants here
#define FET_PERIOD 5
#define DIVIDER_CONSTANT 4
#define IDEAL_DUTY_CYCLE 0.53
#define DUTY_CYCLE_CONSTANT 0.01
#define IDEAL_INPUT_VOLTAGE 9


// Variables belong here
int rawOutVoltage;
float outVoltage;
float integral = 0;
float previousError = 0;
float dutyCycle = 0;
unsigned long lastTime = 0;
unsigned long timeStamp = 0; // Merge with lastTime???

void setup() {
  pinMode(PWMOUT_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  timeStamp = micros();
  digitalWrite(PWMOUT_PIN, HIGH);
  rawOutVoltage = analogRead(VINREG_PIN);

  while (timeStamp - micros() < FET_PERIOD * dutyCycle) {
    delayMicroseconds(1);
  }
  timeStamp = micros();
  outVoltage = ((float)rawOutVoltage / DIVIDER_CONSTANT) / 1024.0;
  digitalWrite(PWMOUT_PIN, LOW);
  while (timeStamp - micros() < FET_PERIOD * (1 - dutyCycle)) {
    delayMicroseconds(1);
  }
  dutyCycle = 0.53;
}




void voltageRegulation(float dutyCycle) {
  digitalWrite(PWMOUT_PIN, HIGH);
  delayMicroseconds(FET_PERIOD * dutyCycle);
  digitalWrite(PWMOUT_PIN, LOW);
  delayMicroseconds(FET_PERIOD * (1 - dutyCycle));
}

float PID_Control(float outVoltage, float P, float I, float D) {
    float error = IDEAL_INPUT_VOLTAGE - outVoltage;
    float dt = micros();
    lastTime = dt;
    dt = micros() - lastTime;
    
    integral += error * dt;
    float derivative = (error - previousError) / dt;
    float output = (P * error) + (I * integral) + (D * derivative);
    previousError = error;

    return output;
}
