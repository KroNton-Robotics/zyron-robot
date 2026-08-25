#include <Arduino.h>
#include "CytronMotorDriver.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// --- Motor Pins ---
constexpr int motorL_PWM = 25;
constexpr int motorL_DIR = 26;
constexpr int motorR_PWM = 12;
constexpr int motorR_DIR = 13;

// --- Encoder Pins ---
constexpr int ENCLA = 4;
constexpr int ENCLB = 16;
constexpr int ENCRA = 18;
constexpr int ENCRB = 19;

// --- Encoder Variables ---
// Marked "volatile" because they change inside hardware interrupts
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// --- Odometry Variables ---
long prev_left_count = 0;
long prev_right_count = 0;

// TODO: Change this to match your specific motor's Pulses Per Revolution!
constexpr float LEFT_TICKS_PER_REVOLUTION = 228.0;
constexpr float RIGHT_TICKS_PER_REVOLUTION = 224.0;

CytronMD motor_left(PWM_DIR, motorL_PWM, motorL_DIR);
CytronMD motor_right(PWM_DIR, motorR_PWM, motorR_DIR);

double RM_RPS_output = 0.0;
double LM_RPS_output = 0.0;
int LM_pwm_output = 0;
int RM_pwm_output = 0;
double pwm_multiplier;
int max_pwm;
MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];

// IMU variables
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
float euler[3];

// Global variables to store the latest IMU data
float latest_yaw = 0.0;
float latest_pitch = 0.0;
float latest_roll = 0.0;
float latest_ax = 0.0;
float latest_ay = 0.0;
float latest_az = 0.0;

// Timing
unsigned long last_time = 0;
unsigned long last_serial_time = 0;  // Watchdog: tracks last serial reception

// Function Prototypes
void motorsSetup();
void sendFeedback(unsigned long current_time);
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();
void setupIMU();
void set_motor_specs(double max_rpm, int max_pwm = 255);
int calculate_pwm(double target_rads);
void updateIMU();

void setup()
{
  Serial.begin(115200);

  Serial.setTimeout(10);
  setupIMU();
  motorsSetup();
  set_motor_specs(280, 255);
  last_serial_time = millis();  // Initialize watchdog so motors don't false-trigger on boot
}

void loop()
{
  // Read all available chunks separated by commas
  while (Serial.available())
  {
    // If ROS sends: "rp150.00,ln050.00,"
    // 1st loop reads: "rp150.00"
    // 2nd loop reads: "ln050.00"
    String chunk = Serial.readStringUntil(',');
    chunk.trim(); // Remove any accidental spaces or hidden characters

    // Make sure the chunk is long enough to be valid (e.g., "rp5.0")
    if (chunk.length() >= 3)
    {
      char motor = chunk.charAt(0); // 'r' or 'l'
      char sign = chunk.charAt(1);  // 'p' or 'n'

      // Extract the numbers after the prefix and convert to integer
      double RPS_val = chunk.substring(2).toDouble();

      // Apply the negative sign if moving backwards
      if (sign == 'n')
      {
        RPS_val = -RPS_val;
      }

      // Assign to the correct motor
      if (motor == 'r')
      {
        RM_RPS_output = RPS_val;
      }
      else if (motor == 'l')
      {
        LM_RPS_output = RPS_val;
      }

      // Update watchdog timestamp on valid command
      last_serial_time = millis();
    }
  }

  // Watchdog: if no serial data received for 500ms, stop motors
  if (millis() - last_serial_time > 500)
  {
    RM_RPS_output = 0.0;
    LM_RPS_output = 0.0;
  }

  // Update motor speeds continually
  LM_pwm_output = constrain(calculate_pwm(LM_RPS_output), -max_pwm, max_pwm); 
  RM_pwm_output = constrain(calculate_pwm(RM_RPS_output), -max_pwm, max_pwm);  

  motor_left.setSpeed(LM_pwm_output);
  motor_right.setSpeed(RM_pwm_output);
  updateIMU();

  // SEND ENCODER FEEDBACK (Runs at 20Hz / every 50ms)
  unsigned long current_time = millis();
  if (current_time - last_time >= 50)
  {
    sendFeedback(current_time);
    last_time = current_time;
  }
  delay(10);
}

void motorsSetup()
{

  // Using INPUT_PULLUP prevents encoder values from floating and causing noise
  pinMode(ENCLA, INPUT_PULLUP);
  pinMode(ENCLB, INPUT_PULLUP);
  pinMode(ENCRA, INPUT_PULLUP);
  pinMode(ENCRB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCLA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCRA), rightEncoderISR, RISING);
}

void setupIMU()
{
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (mpu.dmpInitialize() == 0)
  {
    // supply your own gyro offsets here
    mpu.setXGyroOffset(-15);
    mpu.setYGyroOffset(20);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1788);

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu.setDMPEnabled(true);
    dmpReady = true;
  }
}

void updateIMU()
{
  if (!dmpReady)
    return;

  // Read latest packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    // Get Orientation (gravity-referenced yaw/pitch/roll)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(euler, &q, &gravity);

    latest_yaw = euler[0];
    latest_pitch = euler[1];
    latest_roll = euler[2];

    // Get Acceleration
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    latest_ax = aaReal.x / 10.0;
    latest_ay = aaReal.y / 10.0;
    latest_az = aaReal.z / 10.0;
  }
}

// --- Frame Builder Functions ---

// 1. Dedicated function to build ONLY the encoder string
String buildEncoderFrame(float right_vel, float left_vel)
{
  String frame = "";

  // Right Motor
  frame += "r";
  frame += (right_vel >= 0) ? 'p' : 'n';
  frame += String(abs(right_vel), 2);

  // Left Motor
  frame += ",l";
  frame += (left_vel >= 0) ? 'p' : 'n';
  frame += String(abs(left_vel), 2);

  // Delimiter
  frame += ",";

  return frame;
}

String buildImuFrame()
{
  String frame = "";
  // y = Yaw, p = Pitch, r = Roll
  frame += "y" + String(latest_yaw, 4) + ",";
  frame += "p" + String(latest_pitch, 4) + ",";
  frame += "r" + String(latest_roll, 4) + ",";

  // ax = Accel X, ay = Accel Y, az = Accel Z
  frame += "ax" + String(latest_ax, 2) + ",";
  frame += "ay" + String(latest_ay, 2) + ",";
  frame += "az" + String(latest_az, 2) + ",";

  return frame;
}

// --- Feedback Manager ---

void sendFeedback(unsigned long current_time)
{
  // 1. Safely copy the encoder counts
  noInterrupts();
  long current_left = left_encoder_count;
  long current_right = right_encoder_count;
  interrupts();

  // 2. Calculate time difference in seconds
  float dt = (current_time - last_time) / 1000.0;

  // 3. Calculate change in ticks since last loop
  long delta_left = current_left - prev_left_count;
  long delta_right = current_right - prev_right_count;

  // 4. Calculate Velocity in Radians per Second
  float left_rad_per_sec = (delta_left / LEFT_TICKS_PER_REVOLUTION) * (2.0 * PI) / dt;
  float right_rad_per_sec = (delta_right / RIGHT_TICKS_PER_REVOLUTION) * (2.0 * PI) / dt;

  // 5. Build the individual data frames
  String encoder_msg = buildEncoderFrame(right_rad_per_sec, left_rad_per_sec);
  String imu_msg = buildImuFrame();

  // In the future, you will just add your IMU frame here:
  // String imu_msg = buildImuFrame(imu.roll, imu.pitch, imu.yaw);
  // String full_msg = encoder_msg + imu_msg;
  String full_msg = encoder_msg + imu_msg;
  // 6. Send the combined message to ROS
  Serial.println(full_msg);

  // 7. Update previous values for the next loop calculation
  prev_left_count = current_left;
  prev_right_count = current_right;
}

// 1. Setup Phase: Run this once during on_init() or on_configure()
void set_motor_specs(double max_rpm, int in_max_pwm)
{
  max_pwm = in_max_pwm;

  // Convert input RPM to rad/s
  double max_rads = max_rpm * (2.0 * M_PI / 60.0);

  // Calculate and store the conversion multiplier
  pwm_multiplier = max_pwm / max_rads;
}

// 2. Control Phase: Run this at 50Hz+ in your write() loop
int calculate_pwm(double target_rads)
{
  // Use the pre-calculated multiplier to save CPU cycles
  int pwm = target_rads * pwm_multiplier;

  // For backward movement, the Cytron PWM_DIR mode needs the complement
  // so that low speed = low duty cycle (not inverted)
  if (pwm < 0)
  {
    pwm = -(max_pwm - abs(pwm));
  }

  return pwm;
}

void IRAM_ATTR leftEncoderISR()
{
  // Quadrature logic: When Phase A rises, check Phase B to determine direction
  if (digitalRead(ENCLB) == HIGH)
  {
    left_encoder_count++;
  }
  else
  {
    left_encoder_count--;
  }
}

void IRAM_ATTR rightEncoderISR()
{
  if (digitalRead(ENCRB) == HIGH)
  {
    right_encoder_count++;
  }
  else
  {
    right_encoder_count--;
  }
}