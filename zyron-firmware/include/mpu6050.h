#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorInt16 gg;          // raw gyro from DMP
VectorFloat gravity;
float euler[3];          // [psi, theta, phi]  Euler angles (radians)
float ypr[3];            // [yaw, pitch, roll]

float imu_data[6];       // [yaw, pitch, roll, ax, ay, az]  — filled by readIMU()

// Angular velocity (rad/s) converted from raw DMP gyro output
float imu_gx = 0.0f, imu_gy = 0.0f, imu_gz = 0.0f;

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setupIMU()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(-15);
  mpu.setYGyroOffset(20);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(1788);

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  mpu.setDMPEnabled(true);

  mpuIntStatus = mpu.getIntStatus();
  dmpReady     = true;
  packetSize   = mpu.dmpGetFIFOPacketSize();
}

// Read one DMP FIFO packet (non-blocking).
// Updates euler[], imu_data[], imu_gx/gy/gz when a packet is available.
void readIMU()
{
  if (!dmpReady)
    return;

  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    return;

  // Euler angles (radians) — yaw, pitch, roll
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);

  imu_data[0] = euler[0];
  imu_data[1] = euler[1];
  imu_data[2] = euler[2];

  // Gravity-free linear acceleration (raw sensor units / 10)
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  imu_data[3] = aaReal.x / 10.0f;
  imu_data[4] = aaReal.y / 10.0f;
  imu_data[5] = aaReal.z / 10.0f;

  // Angular velocity: raw DMP gyro → rad/s  (±2000 dps, 16.4 LSB/dps)
  mpu.dmpGetGyro(&gg, fifoBuffer);
  const float gyro_scale = (2000.0f / 32768.0f) * (PI / 180.0f);
  imu_gx = gg.x * gyro_scale;
  imu_gy = gg.y * gyro_scale;
  imu_gz = gg.z * gyro_scale;

  blinkState = !blinkState;
}
