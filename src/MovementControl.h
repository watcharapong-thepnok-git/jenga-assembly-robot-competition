#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

struct imu_heading_data {
  float x_heading_deg = 0;
  float y_heading_deg = 0;
  float z_heading_deg = 0;
};


class imu_heading {
private:
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire2);

  imu_heading_data imu_data;
  float x_heading_deg_offset = 0;

  uint32_t update_time = 0;

public:
  void imu_setup();
  void update_heading();

  void set_correct_heading_x(float correct_heading_x_deg);

  float get_heading_x();
  float get_heading_y();
  float get_heading_z();
};


struct EncoderPulse {
  unsigned long period = 0;
  bool direction = 0;
};


class EncoderReader {
private:
  static const uint8_t BUFFER_SIZE = 16;
  static const uint8_t BUFFER_MASK = BUFFER_SIZE - 1;

  uint8_t pin_a = 0;
  uint8_t pin_b = 0;

  volatile EncoderPulse pulse_buffer[BUFFER_SIZE];
  volatile uint8_t write_idx = 0;
  volatile unsigned long rising_time_previous = 0;

  volatile int32_t tick_count = 0;

  uint8_t read_idx = 0;

  float encoder_speed_rpm = 0;
  float motor_speed_rpm = 0;
  int rotation_direction = 0;

  bool sample_index = 0;
  float motor_rpm_sample[2] = { 0 };
  float filtered_motor_speed_rpm = 0;

  volatile uint32_t zero_speed_updatetime = 0;

  float encoder_rotation_angle_in_rising_period = 360.0f / 11.0f;  /* (DEG) */ 
  float gear_rotation_angle_scaling = 1.0f / 34.0f;
  float deg_per_sec_to_rpm = 60.0f / 360.0f;
  float micro_sec_to_sec = 0.000001f;

public:
  EncoderReader(uint8_t pin_a, uint8_t pin_b);

  void setup();
  void handle_interrupt();
  void process();

  float get_encoder_speed_rpm();
  float get_motor_speed_rpm();
  float get_filtered_motor_speed_rpm();

  int32_t get_tick_count();
};


class MotorControl {
private:
  uint8_t in1_pwm_pin = 0;
  uint8_t in2_pwm_pin = 0;

  float kp = 0;
  float ki = 0;
  float kd = 0;

  /* Instance-specific PID memory */ 
  float integral = 0;
  float prev_error = 0;
  float pid_out = 0;

public:
  MotorControl(uint8_t in1_pin, uint8_t in2_pin, float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f);

  void setup();
  void spin(int16_t command_duty_cycle);
  void drive(float target_rpm, float current_rpm);
  void reset_pid();
};


struct MovementSystemConfig {
  /* Motor Driver Pins */ 
  uint8_t drv_in1_l = 0;
  uint8_t drv_in2_l = 0;
  uint8_t drv_in1_r = 0;
  uint8_t drv_in2_r = 0;
  uint8_t drv_in1_b = 0;
  uint8_t drv_in2_b = 0;

  /* PID Parameters (Left) */ 
  float kp_l = 0;
  float ki_l = 0;
  float kd_l = 0;

  /* PID Parameters (Right) */ 
  float kp_r = 0;
  float ki_r = 0;
  float kd_r = 0;

  /* PID Parameters (Back) */ 
  float kp_b = 0;
  float ki_b = 0;
  float kd_b = 0;

  /* Encoder Pins */ 
  uint8_t encoder_pin_a_l = 0;
  uint8_t encoder_pin_b_l = 0;
  uint8_t encoder_pin_a_r = 0;
  uint8_t encoder_pin_b_r = 0;
  uint8_t encoder_pin_a_b = 0;
  uint8_t encoder_pin_b_b = 0;

  /* Physical and System Constants */ 
  float maximum_allow_wheel_speed = 0;
  float wheel_radius = 0;
  float radius_center_to_wheel = 0;
  uint32_t control_interval = 0;
};


class MovementControl {
private:
  MotorControl motor_l;
  MotorControl motor_r;
  MotorControl motor_b;

  EncoderReader encoder_l;
  EncoderReader encoder_r;
  EncoderReader encoder_b;

  imu_heading imu;

  float cmd_x_velocity = 0;
  float cmd_y_velocity = 0;
  float cmd_angular_velocity = 0;
  
  float maximum_allow_wheel_speed = 0;
  float radius_center_to_wheel = 0;
  float wheel_radius = 0;

  uint32_t update_time = 0;
  uint32_t control_interval = 0;  /* in microsecond */

  float odometry_pos_x_current = 0;
  float odometry_pos_y_current = 0;

  int32_t prev_tick_l = 0;
  int32_t prev_tick_r = 0;
  int32_t prev_tick_b = 0;
  
  uint32_t odometry_update_time = 0;
  uint32_t odometry_update_time_interval = 1000;

  bool odometry_setup_flag = false;

  void update_odometry(float rpm_left_wheel, float rpm_right_wheel, float rpm_back_wheel, float imu_heading_x);
  void update_odometry(int32_t delta_ticks_l, int32_t delta_ticks_r, int32_t delta_ticks_b, float imu_heading_x);

public:
  MovementControl(MovementSystemConfig config);

  void setup();
  void setup_odometry(float odometry_pos_x_current, float odometry_pos_y_current);

  void update_heading();
  void update_encoder();

  void robot_movement_controller();
  void robot_encoder_l_isr();
  void robot_encoder_r_isr();
  void robot_encoder_b_isr();

  float motor_test_drive(float rpm, char id);
  float encoder_test_read_motor_rpm(char id);

  float get_motor_l_rpm();
  float get_motor_r_rpm();
  float get_motor_b_rpm();

  float get_motor_l_filtered_rpm();
  float get_motor_r_filtered_rpm();
  float get_motor_b_filtered_rpm();

  void robot_movement_cmd(float robot_x_velocity, float robot_y_velocity, float robot_angular_velocity);

  float get_imu_heading_x();
  float get_imu_heading_y();
  float get_imu_heading_z();

  void set_odometry_pos_x_current(float odometry_pos_x_current);
  void set_odometry_pos_y_current(float odometry_pos_y_current);

  float get_odometry_pos_x_current();
  float get_odometry_pos_y_current();
};

#endif