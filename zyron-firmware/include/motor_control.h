
// Pin definitions for motor control
const int motorBL1 = 25; // Back-left motor positive
const int motorBL2 = 26; // Back-left motor negative

const int motorBR1 = 12; // Back-right motor positive
const int motorBR2 = 13; // Back-right motor negative

CytronMD motor_left(PWM_DIR, motorBL1, motorBL2);  //  PWM 1 = motorBL1, DIR 1 = motorBL2.
CytronMD motor_right(PWM_DIR, motorBR1, motorBR2); //  PWM 1 = motorBR1, DIR 1 = motorBR2 .

//****** Left Motor Encoder Pins **********
#define ENCLA 4  // YELLOW
#define ENCLB 16 // GREEN

//****** Right Motor Encoder Pins **********
#define ENCRA 18 // YELLOW
#define ENCRB 19 // GREEN

unsigned long lastInterruptTimeR = 0;
unsigned long lastInterruptTimeL = 0;

float count_per_rev_lm = 228.0; // <======== set this number according to your motor (count per rev) for left motor
float count_per_rev_rm = 224.0; // <======== set this number according to your motor (count per rev) for right motor

unsigned long lastTime, now;

// LEFT MOTOR
volatile int posi_lm = 0;
volatile long LM_encoderPos = 0, LM_last_pos = 0;

// RIGHT MOTOR
volatile int posi_rm = 0;
volatile long RM_encoderPos = 0, RM_last_pos = 0;

// Direct PWM commands received from host  L<pwm>,R<pwm>\n
int LM_direct_pwm = 0;
int RM_direct_pwm = 0;

// Odometry outputs (computed in motor_loop, read by main for serial output)
float pos_left_rad   = 0.0f;
float vel_left_rad_s = 0.0f;
float pos_right_rad  = 0.0f;
float vel_right_rad_s = 0.0f;

void setMotorPWM(int left_pwm, int right_pwm)
{
  LM_direct_pwm = constrain(left_pwm,  -255, 255);
  RM_direct_pwm = constrain(right_pwm, -255, 255);
}

void readEncoder_left();
void readEncoder_right();

void motorsSetup()
{
  pinMode(ENCLA, INPUT);
  pinMode(ENCLB, INPUT);
  pinMode(ENCRA, INPUT);
  pinMode(ENCRB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCLA), readEncoder_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCRA), readEncoder_right, RISING);
}

void motor_loop()
{
  // Debounced encoder sampling
  unsigned long interruptTimeL = millis();
  if (interruptTimeL - lastInterruptTimeL > 8)
  {
    LM_encoderPos = posi_lm;
    lastInterruptTimeL = interruptTimeL;
  }
  unsigned long interruptTimeR = millis();
  if (interruptTimeR - lastInterruptTimeR > 8)
  {
    RM_encoderPos = posi_rm;
    lastInterruptTimeR = interruptTimeR;
  }

  now = millis();

  // Update odometry at ~5 Hz (200 ms)
  if ((now - lastTime) >= 200)
  {
    float dt_s = (now - lastTime) / 1000.0f;

    pos_left_rad    = LM_encoderPos / count_per_rev_lm * TWO_PI;
    vel_left_rad_s  = (LM_encoderPos - LM_last_pos) / count_per_rev_lm * TWO_PI / dt_s;

    pos_right_rad   = RM_encoderPos / count_per_rev_rm * TWO_PI;
    vel_right_rad_s = (RM_encoderPos - RM_last_pos) / count_per_rev_rm * TWO_PI / dt_s;

    LM_last_pos = LM_encoderPos;
    RM_last_pos = RM_encoderPos;
    lastTime    = now;
  }

  // Apply direct PWM commands from host
  motor_left.setSpeed(LM_direct_pwm);
  motor_right.setSpeed(RM_direct_pwm);
}

void readEncoder_left()
{
  int b = digitalRead(ENCLB);
  if (b > 0)
    posi_lm++;
  else
    posi_lm--;
}

void readEncoder_right()
{
  int b = digitalRead(ENCRB);
  if (b > 0)
    posi_rm++;
  else
    posi_rm--;
}
