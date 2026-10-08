#include "MovementControl.h"

/* IMU Implementation */ 

void imu_heading::imu_setup() {
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) {}
  }
}

void imu_heading::update_heading() {
  if (micros() - update_time >= 10000) {
    imu::Quaternion quaternion = bno.getQuat();
    imu::Vector<3> euler = quaternion.toEuler(); /* Right hand rule */

    imu_data.x_heading_deg = euler.x() * RAD_TO_DEG; 
    imu_data.y_heading_deg = euler.y() * RAD_TO_DEG; 
    imu_data.z_heading_deg = euler.z() * RAD_TO_DEG;

    update_time = micros();
  }
}

void imu_heading::set_correct_heading_x(float correct_heading_x_deg) { 
  x_heading_deg_offset = correct_heading_x_deg - imu_data.x_heading_deg; 
  x_heading_deg_offset = x_heading_deg_offset - 360.0f * ceilf((x_heading_deg_offset - 180.0f) / 360.0f); /* (-180, 180] */
}

float imu_heading::get_heading_x() { 
  float correct_heading_x_deg = imu_data.x_heading_deg + x_heading_deg_offset;
  return correct_heading_x_deg - 360.0f * ceilf((correct_heading_x_deg - 180.0f) / 360.0f); /* (-180, 180] */
}

float imu_heading::get_heading_y() { return imu_data.y_heading_deg; }
float imu_heading::get_heading_z() { return imu_data.z_heading_deg; }



/* EncoderReader Implementation */ 

EncoderReader::EncoderReader(uint8_t pin_a, uint8_t pin_b)
  : pin_a(pin_a),
    pin_b(pin_b) {
}

void EncoderReader::setup() {
  pinMode(pin_a, INPUT_PULLDOWN);
  pinMode(pin_b, INPUT_PULLDOWN);
}

void EncoderReader::handle_interrupt() {  /* pin_a */ 
  unsigned long current_time = micros();
  unsigned long duration = current_time - rising_time_previous;

  /* Rejects electrical noise spikes/glitches faster than period 300 microseconds */ 
  if (duration < 300) {
    return;
  }

  bool direction_state = digitalRead(pin_b);

  if (direction_state) {
    tick_count++;
  } else {
    tick_count--;
  }

  uint8_t next_write = write_idx & BUFFER_MASK;

  pulse_buffer[next_write].period = current_time - rising_time_previous;
  pulse_buffer[next_write].direction = direction_state;

  rising_time_previous = current_time;
  write_idx++;
}

void EncoderReader::process() {
  unsigned long total_period_sum = 0;

  uint8_t pulse_count = 0;
  int last_direction = 0;

  uint8_t snapshot_write_idx = write_idx;

  uint32_t current_time = micros();

  while (read_idx != snapshot_write_idx) {
    uint8_t current_read = read_idx & BUFFER_MASK;

    last_direction = pulse_buffer[current_read].direction ? 1 : -1;
    total_period_sum += pulse_buffer[current_read].period;
    pulse_count++;

    read_idx++;
  }

  if (pulse_count > 0) {
    float period = (float)total_period_sum / pulse_count;

    encoder_speed_rpm = (encoder_rotation_angle_in_rising_period / ((float) (period) * micro_sec_to_sec)) * deg_per_sec_to_rpm;
    motor_speed_rpm = encoder_speed_rpm * gear_rotation_angle_scaling;
 
    rotation_direction = last_direction;

    /* Moving average filter (2 Sample) */ 
    motor_rpm_sample[sample_index] = motor_speed_rpm;
    filtered_motor_speed_rpm = (motor_rpm_sample[0] + motor_rpm_sample[1]) * 0.5f;
    sample_index = !sample_index;

    zero_speed_updatetime = current_time;

  } else if (current_time - zero_speed_updatetime >= 500000) {

    /* Zero RPM detection */ 
    encoder_speed_rpm = 0.0f;
    motor_speed_rpm = 0.0f;
    filtered_motor_speed_rpm = 0.0f;

    motor_rpm_sample[0] = 0.0f;
    motor_rpm_sample[1] = 0.0f;

    rotation_direction = 0;
  }
}

float EncoderReader::get_encoder_speed_rpm() { return encoder_speed_rpm; }
float EncoderReader::get_motor_speed_rpm() { return motor_speed_rpm * rotation_direction; }
float EncoderReader::get_filtered_motor_speed_rpm() { return filtered_motor_speed_rpm * rotation_direction; }

int32_t EncoderReader::get_tick_count() { return tick_count; }



/* MotorControl Implementation */ 

MotorControl::MotorControl(uint8_t in1_pin, uint8_t in2_pin, float kp_, float ki_, float kd_)
  : in1_pwm_pin(in1_pin),
    in2_pwm_pin(in2_pin),
    kp(kp_),
    ki(ki_),
    kd(kd_) {
}

void MotorControl::setup() {
  pinMode(in1_pwm_pin, OUTPUT);
  pinMode(in2_pwm_pin, OUTPUT);
}

void MotorControl::spin(int16_t command_duty_cycle) {
  uint16_t max_duty = 255;  /* 8-bit PWM */ 
  uint16_t duty_cycle = abs(command_duty_cycle);

  if (duty_cycle > max_duty) { duty_cycle = max_duty; }

  uint16_t in1_pwm_duty = 0;
  uint16_t in2_pwm_duty = 0;

  if (command_duty_cycle > 0) {
    in1_pwm_duty = duty_cycle;
    in2_pwm_duty = 0;

  } else if (command_duty_cycle < 0) {
    in1_pwm_duty = 0;
    in2_pwm_duty = duty_cycle;
  }

  analogWrite(in1_pwm_pin, in1_pwm_duty);
  analogWrite(in2_pwm_pin, in2_pwm_duty);
}

void MotorControl::drive(float target_rpm, float current_rpm) {
  float error_rpm = target_rpm - current_rpm;

  /* Proportional term */ 
  float p_out = error_rpm * kp;

  /* Derivative term */ 
  float derivative = error_rpm - prev_error;
  float d_out = derivative * kd;

  float next_integral = integral + error_rpm;

  if (ki > 0.0f) {
    float max_integral = 50.0f / ki;
    if (next_integral > max_integral) {
      next_integral = max_integral;
    } else if (next_integral < -max_integral) {
      next_integral = -max_integral;
    }
  } else {
    next_integral = 0.0f;
  }
  
  float i_out_temp = next_integral * ki;
  float pid_out_temp = p_out + i_out_temp + d_out;

  bool saturation_min = (pid_out_temp < -255.0f);
  bool saturation_max = (pid_out_temp > 255.0f);

  if (!(saturation_min && error_rpm < 0) && !(saturation_max && error_rpm > 0)) {
    integral = next_integral;
  }

  pid_out_temp = p_out + (integral * ki) + d_out;
  
  if (pid_out_temp > 255.0f) {
    pid_out_temp = 255.0f;
  } else if (pid_out_temp < -255.0f) {
    pid_out_temp = -255.0f;
  }

  pid_out = (int16_t)(pid_out_temp);
  prev_error = error_rpm;

  spin(pid_out);
}

void MotorControl::reset_pid() {
  integral = 0.0f;
  prev_error = 0.0f;
}



/* MovementControl Implementation */ 

MovementControl::MovementControl(MovementSystemConfig config)
  : motor_l(config.drv_in1_l, config.drv_in2_l, config.kp_l, config.ki_l, config.kd_l),
    motor_r(config.drv_in1_r, config.drv_in2_r, config.kp_r, config.ki_r, config.kd_r),
    motor_b(config.drv_in1_b, config.drv_in2_b, config.kp_b, config.ki_b, config.kd_b),
    encoder_l(config.encoder_pin_a_l, config.encoder_pin_b_l),
    encoder_r(config.encoder_pin_a_r, config.encoder_pin_b_r),
    encoder_b(config.encoder_pin_a_b, config.encoder_pin_b_b),
    maximum_allow_wheel_speed(config.maximum_allow_wheel_speed),
    radius_center_to_wheel(config.radius_center_to_wheel),
    wheel_radius(config.wheel_radius),
    control_interval(config.control_interval) {
}

void MovementControl::setup() {
  motor_l.setup();
  motor_r.setup();
  motor_b.setup();
  
  encoder_l.setup();
  encoder_r.setup();
  encoder_b.setup();

  imu.imu_setup();

  setup_odometry(0.0f, 0.0f);
}

void MovementControl::update_heading() { imu.update_heading(); }

void MovementControl::update_encoder() {
  encoder_l.process();
  encoder_r.process();
  encoder_b.process();

  int32_t current_tick_l = encoder_l.get_tick_count();
  int32_t current_tick_r = encoder_r.get_tick_count();
  int32_t current_tick_b = encoder_b.get_tick_count();

  int32_t delta_tick_l = current_tick_l - prev_tick_l;
  int32_t delta_tick_r = current_tick_r - prev_tick_r;
  int32_t delta_tick_b = current_tick_b - prev_tick_b;
  
  if (delta_tick_l != 0 || delta_tick_r != 0 || delta_tick_b != 0) {
    prev_tick_l = current_tick_l;
    prev_tick_r = current_tick_r;
    prev_tick_b = current_tick_b;

    update_odometry(delta_tick_l, delta_tick_r, delta_tick_b, get_imu_heading_x());
  }

  /*
  if (micros() - odometry_update_time >= odometry_update_time_interval) {
    float left_wheel_velocity  = encoder_l.get_filtered_motor_speed_rpm();
    float right_wheel_velocity = encoder_r.get_filtered_motor_speed_rpm();
    float back_wheel_velocity  = encoder_b.get_filtered_motor_speed_rpm();

    update_odometry(left_wheel_velocity, right_wheel_velocity, back_wheel_velocity, get_imu_heading_x());
  }
  */
}

void MovementControl::robot_movement_controller() {
  if (micros() - update_time >= control_interval) {
    const float sin70 = 0.939692f;
    const float cos70 = 0.342020f;

    float left_wheel_velocity  = encoder_l.get_filtered_motor_speed_rpm();
    float right_wheel_velocity = encoder_r.get_filtered_motor_speed_rpm();
    float back_wheel_velocity  = encoder_b.get_filtered_motor_speed_rpm();

    float rotational_linear_velocity = radius_center_to_wheel * cmd_angular_velocity * DEG_TO_RAD;

    float cmd_left_wheel_linear_velocity  =  (cmd_x_velocity * cos70) + (cmd_y_velocity * sin70) - rotational_linear_velocity;
    float cmd_right_wheel_linear_velocity = -(cmd_x_velocity * cos70) + (cmd_y_velocity * sin70) + rotational_linear_velocity;
    float cmd_back_wheel_linear_velocity  =  (cmd_x_velocity) + rotational_linear_velocity;

    float highest_speed_wheel = fmaxf(fmaxf(fabsf(cmd_left_wheel_linear_velocity), fabsf(cmd_right_wheel_linear_velocity)), fabsf(cmd_back_wheel_linear_velocity));

    if (highest_speed_wheel > maximum_allow_wheel_speed) {
      float max_speed_scaling = maximum_allow_wheel_speed / highest_speed_wheel;

      cmd_left_wheel_linear_velocity  *= max_speed_scaling;
      cmd_right_wheel_linear_velocity *= max_speed_scaling;
      cmd_back_wheel_linear_velocity  *= max_speed_scaling;
    }

    const float RAD_PER_SEC_TO_RPM = 9.549296f;

    float cmd_left_wheel_angular_velocity_rpm  = (cmd_left_wheel_linear_velocity  / wheel_radius) * RAD_PER_SEC_TO_RPM;
    float cmd_right_wheel_angular_velocity_rpm = (cmd_right_wheel_linear_velocity / wheel_radius) * RAD_PER_SEC_TO_RPM;
    float cmd_back_wheel_angular_velocity_rpm  = (cmd_back_wheel_linear_velocity  / wheel_radius) * RAD_PER_SEC_TO_RPM;

    motor_l.drive(cmd_left_wheel_angular_velocity_rpm,  left_wheel_velocity);
    motor_r.drive(cmd_right_wheel_angular_velocity_rpm, right_wheel_velocity);
    motor_b.drive(cmd_back_wheel_angular_velocity_rpm,  back_wheel_velocity);

    update_time = micros();
  }
}

float MovementControl::motor_test_drive(float rpm, char id) {
  float measure_rpm = 0;

  if (id == 'l') {
    encoder_l.process();
    measure_rpm = encoder_l.get_filtered_motor_speed_rpm();
  } else if (id == 'r') {
    encoder_r.process();
    measure_rpm = encoder_r.get_filtered_motor_speed_rpm();
  } else if (id == 'b') {
    encoder_b.process();
    measure_rpm = encoder_b.get_filtered_motor_speed_rpm();
  }

  if (micros() - update_time >= control_interval) {
    if (id == 'l') {
      motor_l.drive(rpm, measure_rpm);
    } else if (id == 'r') {
      motor_r.drive(rpm, measure_rpm);
    } else if (id == 'b') {
      motor_b.drive(rpm, measure_rpm);
    }
    update_time = micros();
  }

  return measure_rpm;
}

float MovementControl::encoder_test_read_motor_rpm(char id) {
  if (id == 'l') {
    encoder_l.process();
    return encoder_l.get_motor_speed_rpm();
  } else if (id == 'r') {
    encoder_r.process();
    return encoder_r.get_motor_speed_rpm();
  } else if (id == 'b') {
    encoder_b.process();
    return encoder_b.get_motor_speed_rpm();
  }

  return 0;
}

float MovementControl::get_motor_l_rpm() { return encoder_l.get_motor_speed_rpm(); }
float MovementControl::get_motor_r_rpm() { return encoder_r.get_motor_speed_rpm(); }
float MovementControl::get_motor_b_rpm() { return encoder_b.get_motor_speed_rpm(); }

float MovementControl::get_motor_l_filtered_rpm() { return encoder_l.get_filtered_motor_speed_rpm(); }
float MovementControl::get_motor_r_filtered_rpm() { return encoder_r.get_filtered_motor_speed_rpm(); }
float MovementControl::get_motor_b_filtered_rpm() { return encoder_b.get_filtered_motor_speed_rpm(); }

void MovementControl::robot_encoder_l_isr() { encoder_l.handle_interrupt(); }
void MovementControl::robot_encoder_r_isr() { encoder_r.handle_interrupt(); }
void MovementControl::robot_encoder_b_isr() { encoder_b.handle_interrupt(); }

void MovementControl::robot_movement_cmd(float robot_x_velocity, float robot_y_velocity, float robot_angular_velocity) {
  cmd_x_velocity = robot_x_velocity;
  cmd_y_velocity  = robot_y_velocity;
  cmd_angular_velocity = robot_angular_velocity;
}

float MovementControl::get_imu_heading_x() { return imu.get_heading_x(); }
float MovementControl::get_imu_heading_y() { return imu.get_heading_y(); }
float MovementControl::get_imu_heading_z() { return imu.get_heading_z(); }


void MovementControl::setup_odometry(float odometry_pos_x_current, float odometry_pos_y_current) {
  this->odometry_pos_x_current = odometry_pos_x_current;
  this->odometry_pos_y_current = odometry_pos_y_current;
  this->odometry_setup_flag = true;
}

void MovementControl::set_odometry_pos_x_current(float odometry_pos_x_current) {
  this->odometry_pos_x_current = odometry_pos_x_current;
  this->odometry_update_time = micros();
}

void MovementControl::set_odometry_pos_y_current(float odometry_pos_y_current) {
  this->odometry_pos_y_current = odometry_pos_y_current;
  this->odometry_update_time = micros();
}

void MovementControl::update_odometry(float rpm_left_wheel, float rpm_right_wheel, float rpm_back_wheel, float imu_heading_x_deg) {
  uint32_t current_time = micros();

  if (odometry_setup_flag) {
    odometry_update_time = current_time;
    odometry_setup_flag = false;
    return;
  }

  const float RPM_TO_RAD_PER_SEC = 0.104719755f;
  
  float left_wheel_linear_velocity  = rpm_left_wheel  * RPM_TO_RAD_PER_SEC * wheel_radius;
  float right_wheel_linear_velocity = rpm_right_wheel * RPM_TO_RAD_PER_SEC * wheel_radius;
  float back_wheel_linear_velocity  = rpm_back_wheel  * RPM_TO_RAD_PER_SEC * wheel_radius;

  const float sin70 = 0.939692f;
  const float cos70 = 0.342020f;

  float vx_robot = (left_wheel_linear_velocity - right_wheel_linear_velocity + 2.0f * back_wheel_linear_velocity) / (2.0f * (1.0f + cos70));
  float vy_robot = (left_wheel_linear_velocity + right_wheel_linear_velocity) / (2.0f * sin70);

  float imu_heading_x_rad = imu_heading_x_deg * DEG_TO_RAD;

  float vx_world = vx_robot * cosf(imu_heading_x_rad) - vy_robot * sinf(imu_heading_x_rad);
  float vy_world = vx_robot * sinf(imu_heading_x_rad) + vy_robot * cosf(imu_heading_x_rad);

  float dt = (float)(current_time - odometry_update_time) * 0.000001f;

  odometry_pos_x_current += vx_world * dt;
  odometry_pos_y_current += vy_world * dt;

  odometry_update_time = current_time;
}

void MovementControl::update_odometry(int32_t delta_ticks_l, int32_t delta_ticks_r, int32_t delta_ticks_b, float imu_heading_x_deg) {
  const float TICKS_PER_REV = 11.0f * 34.0f;
  
  float distance_per_tick = (2.0f * M_PI * wheel_radius) / TICKS_PER_REV;

  float d_left  = delta_ticks_l * distance_per_tick;
  float d_right = delta_ticks_r * distance_per_tick;
  float d_back  = delta_ticks_b * distance_per_tick;

  const float sin70 = 0.939692f;
  const float cos70 = 0.342020f;

  float dx_robot = (d_left - d_right + 2.0f * d_back) / (2.0f * (1.0f + cos70));
  float dy_robot = (d_left + d_right) / (2.0f * sin70);

  float imu_heading_x_rad = imu_heading_x_deg * DEG_TO_RAD;

  float dx_world = dx_robot * cosf(imu_heading_x_rad) - dy_robot * sinf(imu_heading_x_rad);
  float dy_world = dx_robot * sinf(imu_heading_x_rad) + dy_robot * cosf(imu_heading_x_rad);

  odometry_pos_x_current += dx_world;
  odometry_pos_y_current += dy_world;
}

float MovementControl::get_odometry_pos_x_current() { return odometry_pos_x_current; }
float MovementControl::get_odometry_pos_y_current() { return odometry_pos_y_current; }