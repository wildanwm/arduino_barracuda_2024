#include <DueTimer.h>



// MOTOR

#define en1 17  // Depan kanan
#define motor1LPWM 3
#define motor1RPWM 2

#define en2 29  // Belakang Kanan. Pin asli 16
#define motor2LPWM 5
#define motor2RPWM 4

#define en3 15  // Belakang Kiri
#define motor3LPWM 7
#define motor3RPWM 6

#define en4 14  // Depan Kiri
#define motor4LPWM 9
#define motor4RPWM 8

#define en5 53  // Penggiring Kiri
#define motor5LPWM 13
#define motor5RPWM 12

#define en6 51  // Penggiring Kanan
#define motor6LPWM 11
#define motor6RPWM 10

#define en7 31  // Penggiring Kanan
#define motor7LPWM A7
#define motor7RPWM A6



// ENCODER

#include <Encoder.h>

#define FR1 28  // front right A
#define FR2 30  // front right B
#define BR1 26  // back right A
#define BR2 32  // back right B
#define BL1 24  // back left A
#define BL2 34  // back left B
#define FL1 22  // front left A
#define FL2 36  // front left B

#define R_ENC1 48
#define R_ENC2 38
#define L_ENC1 46
#define L_ENC2 40

#define encoder_rate 10     // (ms)
#define ppr 537.6           // pulse per rotation dari 19.2 * 7 * 4
#define wheel_radius 51.0   // wheel radius (mm)
#define robot_radius 244.0  // robot radius (mm)
#define pi 3.141592653589793238462643383279502884197

#define ppr_ext 4000           // pulse per rotation encoder external 1000 * 4
#define wheel_radius_ext 34.0  // wheel radius encoder external (mm)
#define enc_radius_ext 25.0    // encoder external radius (mm)

Encoder FR_enc(FR1, FR2);  // (A,B)
Encoder BR_enc(BR1, BR2);
Encoder BL_enc(BL1, BL2);
Encoder FL_enc(FL1, FL2);
Encoder R_enc(R_ENC1, R_ENC2);
Encoder L_enc(L_ENC1, L_ENC2);



// KINEMATIK

#include <pid.h>

int max_pwm = 3500;  // 3500 merupakan hasil dari pembatasan, max_pwm yang asli yaitu 4095
int min_pwm = 500;
float max_w_mtr = 0.27196414654;  // kecepatan motor saat pwm maksimal rad/s * encoder_rate/1000
float w1, w2, w3, w4;
int pwm_w1, pwm_w2, pwm_w3, pwm_w4;
float w1_target, w2_target, w3_target, w4_target;

double set_point1, input1;
double set_point2, input2;
double set_point3, input3;
double set_point4, input4;
double kp1 = 0.000001, ki1 = 0.3, kd1 = 0.4;  // jika ki = 0, pid akan error
double kp2 = 0.000001, ki2 = 0.3, kd2 = 0.4;  // kp = 0.7 or 0.6
double kp3 = 0.000001, ki3 = 0.3, kd3 = 0.4;  // ki = 0.4 or 0.2
double kp4 = 0.000001, ki4 = 0.3, kd4 = 0.4;

epid_t pid_w1;
epid_t pid_w2;
epid_t pid_w3;
epid_t pid_w4;

double w_fr_enc, w_br_enc, w_bl_enc, w_fl_enc;
double w_r_enc, w_l_enc;

double vx_odo = 0, vy_odo = 0, w_odo = 0, vx0_odo = 0, vy0_odo = 0;
double vx_ext = 0, vy_ext = 0, w_ext = 0, vx0_ext = 0, vy0_ext = 0;

double px_odo = 0, py_odo = 0, theta_odo = 0, px0_odo = 0, py0_odo = 0;
double px_ext = 0, py_ext = 0, theta_ext = 0, px0_ext = 0, py0_ext = 0;
double theta_bno = 0;



// Other Sensors

#define shoot_pin 52  // Trigger Penendang

#define shoot_sens_L 44
#define shoot_sens_R 42

bool ball_reached = false;

bool bnoError = true;



// ROS

#define sample_rate_ms 20
//#define USE_USBCON

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <DueTimer.h>
#include <pid.h>

void VelSet(const geometry_msgs::Pose2D& vel_cmd);
void Reset(const std_msgs::Empty& empty);
void ShootBall(const std_msgs::Byte& shoot_power);
void SetInitialPose(const geometry_msgs::Pose2D& pose);

ros::NodeHandle nh;

geometry_msgs::Pose2D robot_pose_msg;
geometry_msgs::Pose2D robot_pose_other_msg;
std_msgs::Bool ball_reached_msg;

std_msgs::Empty reset_feedback_msg;
std_msgs::Byte shoot_feedback_msg;
geometry_msgs::Pose2D initial_feedback_msg;

ros::Subscriber<geometry_msgs::Pose2D> sub1("/robot/vel_target", VelSet);
ros::Subscriber<std_msgs::Empty> sub2("/robot/reset", Reset);
ros::Subscriber<std_msgs::Byte> sub3("/robot/shoot", ShootBall);
ros::Subscriber<geometry_msgs::Pose2D> sub4("/robot/initial_pose", SetInitialPose);

ros::Publisher pose_pub("/robot/pose", &robot_pose_msg);
ros::Publisher pose_other_pub("/robot/pose_other", &robot_pose_other_msg);
ros::Publisher ball_pub("/robot/ball_reached", &ball_reached_msg);

ros::Publisher robot_reset_feedback("/robot/reset/feedback", &reset_feedback_msg);
ros::Publisher robot_shoot_feedback("/robot/shoot/feedback", &shoot_feedback_msg);
ros::Publisher robot_initial_feedback("/robot/initial_pose/feedback", &initial_feedback_msg);

struct timer {
  int time;
  int delta;
} tim;

union init_pose {
  struct parameter {
    float x;
    float y;
    float theta;
  } parameter;
  byte packet[12];
} init_pose;

union data {
  struct parameter {
    float w1;
    float w2;
    float w3;
    float w4;
    float x;
    float y;
    float theta;
    float vel;
  } parameter;
  byte packet[32];
} data;



void setup() {
  Serial.begin(57600);
  delay(100);

  ROSSetup();
  delay(100);

  MotorSetup();
  delay(100);

  Timer4.attachInterrupt(EncoderHandler).start(encoder_rate * 1000);  // call encoderHandler every 10 ms
  delay(100);
  ResetEncoder();
  delay(100);

  PID_setup();
  delay(100);

  ShooterSetup();
  delay(100);

  ForwardKinematics(0, 0, 0);
  SetPenggiring(2000);
}

// int speed_trial = 0;
// bool arah = false;

void loop() {

  // ForwardKinematics(0, speed_trial, 0);
  // if (!arah) {
  //   speed_trial += 100;
  //   if (speed_trial >= 1500) {
  //     arah = true;
  //   }
  // } else {
  //   speed_trial -= 100;
  //   if (speed_trial <= -1500) {
  //     arah = false;
  //   }
  // }

  PID_compute();

  GetBNOData();

  ROSPublish();

  nh.spinOnce();

  while (millis() - tim.time < sample_rate_ms)
    ;
  tim.delta = millis() - tim.time;
  tim.time = millis();
}
