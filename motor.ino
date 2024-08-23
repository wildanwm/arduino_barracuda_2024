void MotorSetup() {
  pinMode(en1, OUTPUT);
  pinMode(motor1LPWM, OUTPUT);
  pinMode(motor1RPWM, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(motor2LPWM, OUTPUT);
  pinMode(motor2RPWM, OUTPUT);
  pinMode(en3, OUTPUT);
  pinMode(motor3LPWM, OUTPUT);
  pinMode(motor3RPWM, OUTPUT);
  pinMode(en4, OUTPUT);
  pinMode(motor4LPWM, OUTPUT);
  pinMode(motor4RPWM, OUTPUT);

  pinMode(en5, OUTPUT);
  pinMode(motor5LPWM, OUTPUT);
  pinMode(motor5RPWM, OUTPUT);
  pinMode(en6, OUTPUT);
  pinMode(motor6LPWM, OUTPUT);
  pinMode(motor6RPWM, OUTPUT);

  pinMode(en7, OUTPUT);
  pinMode(motor7LPWM, OUTPUT);
  pinMode(motor7RPWM, OUTPUT);
}

void setMotor(int PWM1, int PWM2, int PWM3, int PWM4) {
  if (PWM1 < -max_pwm) {  //MOTOR1
    PWM1 = -max_pwm;
  } else if (PWM1 > max_pwm) {
    PWM1 = max_pwm;
  }
  if (PWM1 < 0) {
    PWM1 *= -1;
    digitalWrite(en1, HIGH);
    analogWriteResolution(12);
    analogWrite(motor1LPWM, PWM1);
    analogWrite(motor1RPWM, 0);
  } else if (PWM1 > 0) {
    digitalWrite(en1, HIGH);
    analogWriteResolution(12);
    analogWrite(motor1LPWM, 0);
    analogWrite(motor1RPWM, PWM1);
  } else if (PWM1 == 0) {
    digitalWrite(en1, LOW);
    analogWriteResolution(12);
    analogWrite(motor1LPWM, 0);
    analogWrite(motor1RPWM, 0);
  }

  if (PWM2 < -max_pwm) {  //MOTOR2
    PWM2 = -max_pwm;
  } else if (PWM2 > max_pwm) {
    PWM2 = max_pwm;
  }
  if (PWM2 < 0) {
    PWM2 *= -1;
    digitalWrite(en2, HIGH);
    analogWriteResolution(12);
    analogWrite(motor2LPWM, PWM2);
    analogWrite(motor2RPWM, 0);
  } else if (PWM2 > 0) {
    digitalWrite(en2, HIGH);
    analogWriteResolution(12);
    analogWrite(motor2LPWM, 0);
    analogWrite(motor2RPWM, PWM2);
  } else if (PWM2 == 0) {
    digitalWrite(en2, LOW);
    analogWriteResolution(12);
    analogWrite(motor2RPWM, 0);
    analogWrite(motor2LPWM, 0);
  }

  if (PWM3 < -max_pwm) {  //MOTOR3
    PWM3 = -max_pwm;
  } else if (PWM3 > max_pwm) {
    PWM3 = max_pwm;
  }
  if (PWM3 < 0) {
    PWM3 *= -1;
    digitalWrite(en3, HIGH);
    analogWriteResolution(12);
    analogWrite(motor3LPWM, PWM3);
    analogWrite(motor3RPWM, 0);
  } else if (PWM3 > 0) {
    digitalWrite(en3, HIGH);
    analogWriteResolution(12);
    analogWrite(motor3LPWM, 0);
    analogWrite(motor3RPWM, PWM3);
  } else if (PWM3 == 0) {
    digitalWrite(en3, LOW);
    analogWriteResolution(12);
    analogWrite(motor3LPWM, 0);
    analogWrite(motor3RPWM, 0);
  }

  if (PWM4 < -max_pwm) {  //MOTOR4
    PWM4 = -max_pwm;
  } else if (PWM4 > max_pwm) {
    PWM4 = max_pwm;
  }
  if (PWM4 < 0) {
    PWM4 *= -1;
    digitalWrite(en4, HIGH);
    analogWriteResolution(12);
    analogWrite(motor4LPWM, PWM4);
    analogWrite(motor4RPWM, 0);
  } else if (PWM4 > 0) {
    digitalWrite(en4, HIGH);
    analogWriteResolution(12);
    analogWrite(motor4LPWM, 0);
    analogWrite(motor4RPWM, PWM4);
  } else if (PWM4 == 0) {
    digitalWrite(en4, LOW);
    analogWriteResolution(12);
    analogWrite(motor4LPWM, 0);
    analogWrite(motor4RPWM, 0);
  }
}

void SetPenggiring(int active) {
  analogWriteResolution(12);
  analogWrite(motor5LPWM, 0);
  analogWrite(motor6RPWM, 0);
  if (active > 0) {
    digitalWrite(en5, HIGH);
    analogWrite(motor5RPWM, active);
    digitalWrite(en6, HIGH);
    analogWrite(motor6LPWM, active);
  } else {
    digitalWrite(en5, LOW);
    analogWrite(motor5RPWM, 0);
    digitalWrite(en6, LOW);
    analogWrite(motor6LPWM, 0);
  }
}
