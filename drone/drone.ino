#include <Wire.h>
#include <Servo.h>
#include <MPU6050_light.h>

struct MPUData {
    float x, y, z;
    float yaw, pitch, roll;
};

// will be created for yaw, pitch, and roll controllers
struct PIDGains {
  float p, i, d;
  float net_error;
  float prev_error;
  float set_point;
  float curr;
};

// global variables
float time_prev, time;

Servo front_right, front_left, back_right, back_left;
Servo motors[4] = {front_left, front_right, back_left, back_right};

MPU6050 mpu(Wire);
MPUData mpu_data;

PIDGains yaw_gains, pitch_gains, roll_gains;

void config_mpu(MPU6050 *mpu) {
  Wire.begin();
  byte status = mpu->begin();
  
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu->calcOffsets(true,true); // gyro and accel
  Serial.println("Done!\n");
}

void update_mpu(MPUData *mpu_data, MPU6050 *mpu) {
  mpu_data->pitch = mpu->getAngleX();
  mpu_data->roll = mpu->getAngleY();
  mpu_data->yaw = mpu->getAngleZ();

  /* logic to find x, y, z positions */
}

float calculate_pid_output(PIDGains *gains, float elapsed_time) {
  float error = gains->set_point - gains->curr;
  float pid_p = gains->p * error;
  float pid_d = gains->d * (error - gains->prev_error) / elapsed_time;
  float pid_i = gains->i * (gains->net_error + error);

  gains->prev_error = error;
  gains->net_error += error;

  return pid_p + pid_d + pid_i;
}

void config_motors(Servo motors[4]) {
  motors[0].attach(1);
  motors[1].attach(2);
  motors[2].attach(3);
  motors[3].attach(4);

  motors[0].writeMicroseconds(1000);
  motors[1].writeMicroseconds(1000);
  motors[2].writeMicroseconds(1000);
  motors[3].writeMicroseconds(1000);
  delay(7000);
}

/*
 * Order of motors: front left, front right, back left, back right
 */
void write_to_motors(float thrust, PIDGains *yaw_gains, PIDGains *pitch_gains, PIDGains *roll_gains, Servo motors[4], float elapsed_time) {
  float yaw = calculate_pid_output(yaw_gains, elapsed_time);
  float pitch = calculate_pid_output(pitch_gains, elapsed_time);
  float roll = calculate_pid_output(roll_gains, elapsed_time);
  
  float front_right = thrust + yaw + pitch + roll;
  float front_left = thrust - yaw + pitch - roll;
  float back_right = thrust - yaw - pitch + roll;
  float back_left = thrust + yaw - pitch - roll;
  
  motors[0].writeMicroseconds(front_left);
  motors[1].writeMicroseconds(front_right);
  motors[2].writeMicroseconds(back_left);
  motors[3].writeMicroseconds(back_right);
}

void setup() {
  // put your setup code here, to run once:
  config_mpu(&mpu);
  config_motors(motors);
}

void loop() {
  // put your main code here, to run repeatedly:
  time_prev = time;
  time = millis();
  float elapsed_time = (time - time_prev) / 1000;

  update_mpu(&mpu_data, &mpu);

  /* RC communications to get setpoints */
  /* Set setpoints in pid gains */
  float thrust = 1000; // eventually get from controller
  
  write_to_motors(thrust, &yaw_gains, &pitch_gains, &roll_gains, motors, elapsed_time);

}
