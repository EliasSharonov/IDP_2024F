// Declare pins here
#define PWMOUT_PIN 10
#define VINREG_PIN 9

// Declare other constants here
#define FET_PERIOD 10
#define DIVIDER_CONSTANT 2


// Variables belong here
float dutyCycle;




void setup() {
  pinMode(PWMOUT_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int inputVoltage = analogRead(VINREG_PIN);
  inputVoltage = (inputVoltage / DIVIDER_CONSTANT) / 1024.0;

  dutyCycle = 1 - (inputVoltage / 10.7);
  voltageRegulation(dutyCycle);
}

void voltageRegulation(float dutyCycle) {
  digitalWrite(PWMOUT_PIN, HIGH);
  delayMicroseconds(FET_PERIOD * dutyCycle);
  digitalWrite(PWMOUT_PIN, LOW);
  delayMicroseconds(FET_PERIOD * (1 - dutyCycle));

}
