#include <Arduino.h>
#include "CytronMotorDriver.h"
#include <motor_control.h>
#include <mpu6050.h>

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(50);  // ms — prevents readStringUntil from blocking on partial frames
  motorsSetup();
  setupIMU();
}

void loop()
{
  // 1. Parse incoming command:  L<pwm>,R<pwm>\n
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    int lp, rp;
    if (sscanf(cmd.c_str(), "L%d,R%d", &lp, &rp) == 2)
      setMotorPWM(lp, rp);
  }

  // 2. Update encoder odometry and apply stored PWM to motors
  motor_loop();

  // 3. Read IMU FIFO (non-blocking — updates globals when packet is ready)
  readIMU();

  // 4. Send joint state:  L<pos_rad>,<vel_rad_s>,R<pos_rad>,<vel_rad_s>\n
  Serial.print("L");  Serial.print(pos_left_rad,   4);
  Serial.print(",");  Serial.print(vel_left_rad_s,  4);
  Serial.print(",R"); Serial.print(pos_right_rad,   4);
  Serial.print(",");  Serial.println(vel_right_rad_s, 4);

  // 5. Send IMU:  I<yaw>,<pitch>,<roll>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n
  Serial.print("I");  Serial.print(euler[0], 4);    // yaw   (rad)
  Serial.print(",");  Serial.print(euler[1], 4);    // pitch (rad)
  Serial.print(",");  Serial.print(euler[2], 4);    // roll  (rad)
  Serial.print(",");  Serial.print(imu_data[3], 4); // ax
  Serial.print(",");  Serial.print(imu_data[4], 4); // ay
  Serial.print(",");  Serial.print(imu_data[5], 4); // az
  Serial.print(",");  Serial.print(imu_gx, 4);      // gx (rad/s)
  Serial.print(",");  Serial.print(imu_gy, 4);      // gy (rad/s)
  Serial.print(",");  Serial.println(imu_gz, 4);    // gz (rad/s)

  delay(5); // ~200 Hz loop
}
