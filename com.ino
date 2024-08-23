void ROSSetup() {
  nh.initNode();
  nh.advertise(pose_pub);
  nh.advertise(pose_other_pub);
  nh.advertise(ball_pub);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);

  nh.advertise(robot_reset_feedback);
  nh.advertise(robot_shoot_feedback);
  nh.advertise(robot_initial_feedback);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void ROSPublish() {
  robot_pose_msg.x = px0_odo;
  robot_pose_msg.y = py0_odo;
  if (bnoError) {
    robot_pose_msg.theta = theta_odo;
  } else {
    robot_pose_msg.theta = theta_bno;
  }
  pose_pub.publish(&robot_pose_msg);

  robot_pose_other_msg.x = px0_ext;
  robot_pose_other_msg.y = py0_ext;
  if (bnoError) {
    robot_pose_other_msg.theta = theta_odo;
  } else {
    robot_pose_other_msg.theta = theta_bno;
  }
  pose_other_pub.publish(&robot_pose_other_msg);

  BallCheck();
  ball_reached_msg.data = ball_reached;
  ball_pub.publish(&ball_reached_msg);
}

void VelSet(const geometry_msgs::Pose2D& vel_cmd) {
  ForwardKinematics(vel_cmd.x, vel_cmd.y, vel_cmd.theta);
}

void Reset(const std_msgs::Empty& empty) {
  robot_reset_feedback.publish(&reset_feedback_msg);

  PIDReset();
  ResetEncoder();
}

void ShootBall(const std_msgs::Byte& shoot_power) {
  shoot_feedback_msg.data = shoot_power.data;
  robot_shoot_feedback.publish(&shoot_feedback_msg);

  BoostNShootBall(shoot_power.data);
}

void SetInitialPose(const geometry_msgs::Pose2D& pose) {
  initial_feedback_msg = pose;
  robot_initial_feedback.publish(&initial_feedback_msg);

  init_pose.parameter.x = pose.x;
  init_pose.parameter.y = pose.y;
  init_pose.parameter.theta = pose.theta;

  px_odo = pose.x;
  py_odo = pose.y;
  theta_odo = pose.theta;
  px0_odo = pose.x;
  py0_odo = pose.y;

  px_ext = pose.x;
  py_ext = pose.y;
  // theta_ext = pose.theta;
  px0_ext = pose.x;
  py0_ext = pose.y;
}
