// Declare pins here
#define PWMOUT_PIN 10
#define VINREG_PIN 9

// Declare other constants here
#define FET_PERIOD 10
#define DIVIDER_CONSTANT 4
#define IDEAL_DUTY_CYCLE 0.53
#define DUTY_CYCLE_CONSTANT 0.01
#define IDEAL_INPUT_VOLTAGE 5


// Variables belong here
int rawOutVoltage;
float outVoltage;
float integral = 0;
float previousError = 0;
unsigned long lastTime = 0;


void setup() {
  pinMode(PWMOUT_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  rawOutVoltage = analogRead(VINREG_PIN);
  outVoltage = ((float)rawOutVoltage / DIVIDER_CONSTANT) / 1024.0;

  voltageRegulation(PID_Control(outVoltage, 0.05, 0.05, 0.05));
}




void voltageRegulation(float dutyCycle) {
  digitalWrite(PWMOUT_PIN, HIGH);
  delayMicroseconds(FET_PERIOD * dutyCycle);
  digitalWrite(PWMOUT_PIN, LOW);
  delayMicroseconds(FET_PERIOD * (1 - dutyCycle));
}

float PID_Control(float outVoltage, float P, float I, float D) {
    float error = IDEAL_INPUT_VOLTAGE - outVoltage;
    float dt = millis();
    lastTime = dt;
    dt = millis() - lastTime;
    
    integral += error * dt;
    float derivative = (error - previousError) / dt;
    float output = (P * error) + (I * integral) + (D * derivative);
    previousError = error;

    return output;
}

