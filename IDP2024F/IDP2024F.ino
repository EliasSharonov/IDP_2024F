// We dumped the protothreads library here

#define LC_SET(lc) lc = __LINE__
#define LC_RESUME(lc) switch(lc) { case 0:

#define LC_END(lc) lc = 0; } // End of switch-case

typedef unsigned short lc_t;

struct pt {
  lc_t lc;
};

#define PT_WAITING 0
#define PT_YIELDED 1
#define PT_EXITED  2
#define PT_ENDED   3

#define PT_INIT(pt)   ((pt)->lc = 0)
#define PT_THREAD(name_args) char name_args
#define PT_BEGIN(pt) { LC_RESUME((pt)->lc)
#define PT_END(pt) LC_END((pt)->lc); return PT_ENDED; }

#define PT_WAIT_UNTIL(pt, condition)            \
  do {                                          \
    LC_SET((pt)->lc);                           \
    if(!(condition)) {                          \
      return PT_WAITING;                        \
    }                                           \
  } while(0)

#define PT_WAIT_WHILE(pt, cond)  PT_WAIT_UNTIL((pt), !(cond))

#define PT_WAIT_THREAD(pt, thread) PT_WAIT_WHILE((pt), PT_SCHEDULE(thread))

#define PT_SPAWN(pt, child, thread)             \
  do {                                          \
    PT_INIT((child));                           \
    PT_WAIT_THREAD((pt), (thread));             \
  } while(0)

#define PT_RESTART(pt)                          \
  do {                                          \
    PT_INIT(pt);                                \
    return PT_WAITING;                          \
  } while(0)

#define PT_EXIT(pt)                             \
  do {                                          \
    PT_INIT(pt);                                \
    return PT_EXITED;                           \
  } while(0)

#define PT_SCHEDULE(f) ((f) < PT_EXITED)

#define PT_YIELD(pt)                            \
  do {                                          \
    LC_SET((pt)->lc);                           \
    return PT_YIELDED;                          \
  } while(0)

#define PT_YIELD_UNTIL(pt, cond)                \
  do {                                          \
    LC_SET((pt)->lc);                           \
    if(!(cond)) {                               \
      return PT_YIELDED;                        \
    }                                           \
  } while(0)


// Declare pins here
#define PWMOUT_PIN 10  // PB2
#define VINREG_PIN A0

// Declare other constants here
#define FET_PERIOD 10 //microseconds
#define DIVIDER_CONSTANT 11 // Approximately
#define IDEAL_DUTY_CYCLE 0.53
#define DUTY_CYCLE_CONSTANT 0.01
#define IDEAL_INPUT_VOLTAGE 10.7

// Protothreads
static struct pt protoThread1;  
static struct pt protoThread2;
static struct pt protoThread3;

// Variables
float outVoltage;
float dutyCycle;

// PID stuff so that the board does not spend time initializing them
float error = 0;
float integral = 0;
float previousError = 0;
float dt = 0;
float derivative =  0;

unsigned long lastTime = 0;

static int voltageRegulation(struct pt *protoThread1) {
  PT_BEGIN(protoThread1);

  while(1) {
    OCR1B = (uint16_t)(dutyCycle * ICR1);
    
    PT_YIELD(protoThread1);
  }

  PT_END(protoThread1);
}

static int voltageRead(struct pt *protoThread2) {
  PT_BEGIN(protoThread2);

  while(1) {
    outVoltage = ((float)analogRead(VINREG_PIN) * DIVIDER_CONSTANT) / 203.0;
    
    PT_YIELD(protoThread2);
  }

  PT_END(protoThread2);
}

static int adjustPID(struct pt *protoThread3, float P, float I, float D) {
  PT_BEGIN(protoThread3);

  while(1) {
    error = IDEAL_INPUT_VOLTAGE - outVoltage;
    dt = (micros() - lastTime) / 1000000.0;
    lastTime = micros();
    integral += error * dt;
    derivative = (error - previousError) / dt;
    dutyCycle = IDEAL_DUTY_CYCLE + (P * error) + (I * integral) + (D * derivative);
    
    if (dutyCycle > 1) dutyCycle = 1;
    if (dutyCycle < 0) dutyCycle = 0;
    
    previousError = error;
    
    // Yield control to other threads
    PT_YIELD(protoThread3);
  }

  PT_END(protoThread3);
}

void setup() {
  pinMode(PWMOUT_PIN, OUTPUT);
  pinMode(VINREG_PIN, INPUT);

  PT_INIT(&protoThread1);
  PT_INIT(&protoThread2);
  PT_INIT(&protoThread3);

  noInterrupts();

  TCCR1A = (1 << COM1B1) | (1 << WGM11); 
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);

  ICR1 = 160;

  OCR1B = (uint16_t)(IDEAL_DUTY_CYCLE * ICR1);

  interrupts();
}

void loop() {
  PT_SCHEDULE(voltageRegulation(&protoThread1));
  PT_SCHEDULE(voltageRead(&protoThread2));
  PT_SCHEDULE(adjustPID(&protoThread3, 0.01, 0.1, 0));
}
