void ShooterSetup() {
  pinMode(shoot_pin, OUTPUT);
  digitalWrite(shoot_pin, LOW);

  pinMode(shoot_sens_L, INPUT);
  pinMode(shoot_sens_R, INPUT);
}

void BallCheck() {
  bool l_catch = digitalRead(shoot_sens_L);
  bool r_catch = digitalRead(shoot_sens_R);
  ball_reached = ((!l_catch) || (!r_catch));

  if (ball_reached){
    SetPenggiring(3500);
  }
  else{
    SetPenggiring(1000);
  }

  //  Serial.print(" Left Catch : ");
  //  Serial.print(!l_catch);
  //  Serial.print("    Right Catch : ");
  //  Serial.print(!r_catch);
  //  Serial.print("    Ball Reached : ");
  //  Serial.print(ball_reached);
  //  Serial.println("");
}

void BoostNShootBall(byte power) {
  // Serial.println("WEHHH COKKK");
  bool long_ball = false;

  if (power >= 51) {
    digitalWrite(en7, HIGH);
    analogWriteResolution(12);
    analogWrite(motor7LPWM, 2100);
    analogWrite(motor7RPWM, 0);
    delay(75);
    digitalWrite(en7, HIGH);
    analogWrite(motor7LPWM, 0);
    analogWrite(motor7RPWM, 2100);
    delay(5);
    digitalWrite(en7, LOW);
    analogWrite(motor7LPWM, 0);
    analogWrite(motor7RPWM, 0);
    
    delay(100);

    power -= 50;
    long_ball = true;

    SetPenggiring(0);
    delay(120);
  }
  else{
    SetPenggiring(0);
    delay(200);
  }

  digitalWrite(shoot_pin, HIGH);
  delay(power);
  digitalWrite(shoot_pin, LOW);

  if (long_ball) {
    delay(1250);

    digitalWrite(en7, HIGH);
    analogWriteResolution(12);
    analogWrite(motor7LPWM, 0);
    analogWrite(motor7RPWM, 2100);
    delay(75);
    digitalWrite(en7, HIGH);
    analogWrite(motor7LPWM, 2100);
    analogWrite(motor7RPWM, 0);
    delay(1);
    digitalWrite(en7, LOW);
    analogWrite(motor7LPWM, 0);
    analogWrite(motor7RPWM, 0);

    SetPenggiring(1000);
  }
  else{
    delay(400);
    SetPenggiring(1000);
  }
}
