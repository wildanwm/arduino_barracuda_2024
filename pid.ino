void PID_setup() {
  epid_info_t epid_err_1 = epid_init(&pid_w1, 0.0, 0.0, 0.0, kp1, ki1, kd1);
  epid_info_t epid_err_2 = epid_init(&pid_w2, 0.0, 0.0, 0.0, kp2, ki2, kd2);
  epid_info_t epid_err_3 = epid_init(&pid_w3, 0.0, 0.0, 0.0, kp3, ki3, kd3);
  epid_info_t epid_err_4 = epid_init(&pid_w4, 0.0, 0.0, 0.0, kp4, ki4, kd4);

  set_point1 = 0;
  set_point2 = 0;
  set_point3 = 0;
  set_point4 = 0;

  while ((epid_err_1 != EPID_ERR_NONE) && (epid_err_2 != EPID_ERR_NONE) && (epid_err_3 != EPID_ERR_NONE) && (epid_err_4 != EPID_ERR_NONE)) {
    Serial.println("PID Error");
    delay(500);
  }
  // Serial.println("PID Ready");
}

void PIDReset() {
  ForwardKinematics(0, 0, 0);
  PID_setup();
}

float mapF(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PID_compute() {
  set_point1 = mapF(w1_target, -max_w_mtr, max_w_mtr, -1, 1);
  set_point2 = mapF(w2_target, -max_w_mtr, max_w_mtr, -1, 1);
  set_point3 = mapF(w3_target, -max_w_mtr, max_w_mtr, -1, 1);
  set_point4 = mapF(w4_target, -max_w_mtr, max_w_mtr, -1, 1);

  input1 = mapF(w_fr_enc, -max_w_mtr, max_w_mtr, -1, 1);
  input2 = mapF(w_br_enc, -max_w_mtr, max_w_mtr, -1, 1);
  input3 = mapF(w_bl_enc, -max_w_mtr, max_w_mtr, -1, 1);
  input4 = mapF(w_fl_enc, -max_w_mtr, max_w_mtr, -1, 1);

  epid_pid_calc(&pid_w1, set_point1, input1);
  epid_pid_calc(&pid_w2, set_point2, input2);
  epid_pid_calc(&pid_w3, set_point3, input3);
  epid_pid_calc(&pid_w4, set_point4, input4);

  epid_pid_sum(&pid_w1, -1, 1);
  epid_pid_sum(&pid_w2, -1, 1);
  epid_pid_sum(&pid_w3, -1, 1);
  epid_pid_sum(&pid_w4, -1, 1);

  pwm_w1 = mapF(pid_w1.y_out, -1, 1, -max_pwm, max_pwm);
  pwm_w2 = mapF(pid_w2.y_out, -1, 1, -max_pwm, max_pwm);
  pwm_w3 = mapF(pid_w3.y_out, -1, 1, -max_pwm, max_pwm);
  pwm_w4 = mapF(pid_w4.y_out, -1, 1, -max_pwm, max_pwm);

  // Serial.print(w2_target, 7);
  // Serial.print("    ");
  // Serial.print(w_br_enc, 7);
  // Serial.print("    ");
  // Serial.print(pid_w2.y_out, 7);
  // Serial.print("    ");
  // Serial.print(-pwm_w2, 7);
  // Serial.println("    ");

  setMotor(pwm_w1, pwm_w2, pwm_w3, pwm_w4);
}
