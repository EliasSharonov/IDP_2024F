// Declare pins here
#define PWMOUT_PIN 10
#define VINREG_PIN 9

// Declare other constants here
#define FET_PERIOD 10
#define DIVIDER_CONSTANT 4
#define IDEAL_DUTY_CYCLE 0.53
#define DUTY_CYCLE_CONSTANT 0.01


// Variables belong here
float dutyCycle;
float dutyCycleError;
int rawOutVoltage;
float outVoltage;




void setup() {
  pinMode(PWMOUT_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  rawOutVoltage = analogRead(VINREG_PIN);
  outVoltage = ((float)rawOutVoltage / DIVIDER_CONSTANT) / 1024.0;


  
  dutyCycleError = IDEAL_DUTY_CYCLE - (1.0 - (5.0 / outVoltage));
  dutyCycle = IDEAL_DUTY_CYCLE - DUTY_CYCLE_CONSTANT * dutyCycleError;
  voltageRegulation(dutyCycle);
}

void voltageRegulation(float dutyCycle) {
  digitalWrite(PWMOUT_PIN, HIGH);
  delayMicroseconds(FET_PERIOD * dutyCycle);
  digitalWrite(PWMOUT_PIN, LOW);
  delayMicroseconds(FET_PERIOD * (1 - dutyCycle));

}
