void ForwardKinematics(float v_x, float v_y, float w) {
  w1_target = (-0.707106781 * v_y + 0.707106781 * v_x + w * (pi / 180) * robot_radius) / wheel_radius * (float(encoder_rate) / 1000);  // rad/s * encoder_rate/1000 roda
  w2_target = (-0.707106781 * v_y - 0.707106781 * v_x + w * (pi / 180) * robot_radius) / wheel_radius * (float(encoder_rate) / 1000);
  w3_target = (0.707106781 * v_y - 0.707106781 * v_x + w * (pi / 180) * robot_radius) / wheel_radius * (float(encoder_rate) / 1000);
  w4_target = (0.707106781 * v_y + 0.707106781 * v_x + w * (pi / 180) * robot_radius) / wheel_radius * (float(encoder_rate) / 1000);
}

void EncoderHandler() {
  w_fr_enc = (float)(FR_enc.read() * (2 * pi) / ppr);  // rad/s * encoder_rate/1000 roda
  FR_enc.write(0);
  w_br_enc = (float)(BR_enc.read() * (2 * pi) / ppr);
  BR_enc.write(0);
  w_bl_enc = (float)(BL_enc.read() * (2 * pi) / ppr);
  BL_enc.write(0);
  w_fl_enc = (float)(FL_enc.read() * (2 * pi) / ppr);
  FL_enc.write(0);

  w_r_enc = (float)(R_enc.read() * (2 * pi) / ppr_ext);
  R_enc.write(0);
  w_l_enc = (float)(L_enc.read() * (2 * pi) / ppr_ext);
  L_enc.write(0);

  InverseKinematics();
  KinematicsExternal();

  // Serial.print(w_fr_enc, 7);
  // Serial.print("    ");
  // Serial.print(w_br_enc, 7);
  // Serial.print("    ");
  // Serial.print(w_bl_enc, 7);
  // Serial.print("    ");
  // Serial.print(w_fl_enc, 7);
  // Serial.println("    ");

  // Serial.print(w_r_enc, 7);
  // Serial.print("    ");
  // Serial.print(w_l_enc, 7);
  // Serial.println("    ");
}

void InverseKinematics() {
  vx_odo = (+w_fr_enc - w_br_enc - w_bl_enc + w_fl_enc) * 0.353553391 * wheel_radius;  // mm/s * encoder_rate/1000
  vy_odo = (-w_fr_enc - w_br_enc + w_bl_enc + w_fl_enc) * 0.353553391 * wheel_radius;
  w_odo = (+w_fr_enc + w_br_enc + w_bl_enc + w_fl_enc) / 4 * wheel_radius / robot_radius * (180 / pi);  // deg/s * encoder_rate/1000 robot

  if (bnoError) {
    vx0_odo = cos(theta_odo * pi / 180) * vx_odo + sin(theta_odo * pi / 180) * vy_odo;
    vy0_odo = cos(theta_odo * pi / 180) * vy_odo - sin(theta_odo * pi / 180) * vx_odo;
  } else {
    vx0_odo = cos(theta_bno * pi / 180) * vx_odo + sin(theta_bno * pi / 180) * vy_odo;
    vy0_odo = cos(theta_bno * pi / 180) * vy_odo - sin(theta_bno * pi / 180) * vx_odo;
  }

  px_odo += vx_odo;  // mm buta arah
  py_odo += vy_odo;

  px0_odo += vx0_odo;  // mm
  py0_odo += vy0_odo;
  theta_odo += w_odo;  // deg
  if (theta_odo > 180) theta_odo -= 360;
  else if (theta_odo < -180) theta_odo += 360;

  // Serial.print(vx0_odo, 7);
  // Serial.print("    ");
  // Serial.print(vy0_odo, 7);
  // Serial.print("    ");
  // Serial.print(w_odo, 7);
  // Serial.println("    ");

  // Serial.print(px0_odo, 7);
  // Serial.print("    ");
  // Serial.print(py0_odo, 7);
  // Serial.print("    ");
  // Serial.print(theta_odo, 7);
  // Serial.println("    ");
}

void KinematicsExternal() {
  vx_ext = (w_r_enc + w_l_enc) * 0.707106781 * wheel_radius_ext;
  vy_ext = (w_r_enc - w_l_enc) * 0.707106781 * wheel_radius_ext;
  // w_ext = (-w_r_enc - w_l_enc) / 2 * wheel_radius_ext / enc_radius_ext * (180 / pi);

  if (bnoError) {
    vx0_ext = cos(theta_odo * pi / 180) * vx_ext + sin(theta_odo * pi / 180) * vy_ext;
    vy0_ext = cos(theta_odo * pi / 180) * vy_ext - sin(theta_odo * pi / 180) * vx_ext;
  } else {
    vx0_ext = cos(theta_bno * pi / 180) * vx_ext + sin(theta_bno * pi / 180) * vy_ext;
    vy0_ext = cos(theta_bno * pi / 180) * vy_ext - sin(theta_bno * pi / 180) * vx_ext;
  }

  px_ext += vx_ext;  // mm buta arah
  py_ext += vy_ext;

  px0_ext += vx0_ext;  // mm
  py0_ext += vy0_ext;
  // theta_ext += w_ext;  // deg
  // if (theta_ext > 180) theta_ext -= 360;
  // else if (theta_ext < -180) theta_ext += 360;

  // Serial.print(vx0_ext, 7);
  // Serial.print("    ");
  // Serial.print(vy0_ext, 7);
  // Serial.print("    ");
  // Serial.print(w_odo, 7);
  // Serial.println("    ");

  // Serial.print(px0_ext, 7);
  // Serial.print("    ");
  // Serial.print(py0_ext, 7);
  // Serial.print("    ");
  // Serial.print(theta_odo, 7);
  // Serial.println("    ");
}

void GetBNOData() {
}

void ResetEncoder() {
  FR_enc.write(0);
  BR_enc.write(0);
  BL_enc.write(0);
  FL_enc.write(0);

  px_odo = init_pose.parameter.x;
  py_odo = init_pose.parameter.y;
  theta_odo = init_pose.parameter.theta;
  px0_odo = init_pose.parameter.x;
  py0_odo = init_pose.parameter.y;

  px_ext = init_pose.parameter.x;
  py_ext = init_pose.parameter.y;
  // theta_ext = init_pose.parameter.theta;
  px0_ext = init_pose.parameter.x;
  py0_ext = init_pose.parameter.y;
}
