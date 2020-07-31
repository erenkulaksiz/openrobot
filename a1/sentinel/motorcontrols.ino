/*
void leftMotorControl(int input) {
  int backwardSpeed, forwardSpeed;

  // Limiter
  if (input < 0) {
    input = 0;
  } else if (input > 1023) {
    input = 1023;
  }

  // Drive backwards
  if (input < 512) {
    backwardSpeed = map(input, 512, 0, 0, 110);
    if (backwardSpeed > 10) {
      analogWrite(SOL_MOTOR_PIN_A, backwardSpeed);
      digitalWrite(SOL_MOTOR_PIN_B, LOW);
    } else {
      digitalWrite(SOL_MOTOR_PIN_A, LOW);
      digitalWrite(SOL_MOTOR_PIN_B, LOW);
    }
  } else if (input > 512) {
    // Drive forward
    forwardSpeed = map(input, 512, 1023, 0, 110);
    if (forwardSpeed > 20) {
      analogWrite(SOL_MOTOR_PIN_B, forwardSpeed);
      digitalWrite(SOL_MOTOR_PIN_A, LOW);
    } else {
      digitalWrite(SOL_MOTOR_PIN_A, LOW);
      digitalWrite(SOL_MOTOR_PIN_B, LOW);
    }
  } else {
    digitalWrite(SOL_MOTOR_PIN_A, LOW);
    digitalWrite(SOL_MOTOR_PIN_B, LOW);
  }
}

void rightMotorControl(int input) {
  int backwardSpeed, forwardSpeed;

  // Limiter
  if (input < 0) {
    input = 0;
  } else if (input > 1023) {
    input = 1023;
  }

  // Backwards
  if (girdi < 512) {
    backwardSpeed = map(input, 512, 0, 0, 110);
    if (backwardSpeed > 10) {
      analogWrite(SAG_MOTOR_PIN_A, backwardSpeed);
      digitalWrite(SAG_MOTOR_PIN_B, LOW);
    } else {
      digitalWrite(SAG_MOTOR_PIN_A, LOW);
      digitalWrite(SAG_MOTOR_PIN_B, LOW);
    }
  } else if (input > 512) {
    // Forwards
    forwardSpeed = map(input, 512, 1023, 0, 110);
    if (forwardSpeed > 20) {
      analogWrite(SAG_MOTOR_PIN_B, forwardSpeed);
      digitalWrite(SAG_MOTOR_PIN_A, LOW);
    } else {
      digitalWrite(SAG_MOTOR_PIN_A, LOW);
      digitalWrite(SAG_MOTOR_PIN_B, LOW);
    }
  } else {
    digitalWrite(SAG_MOTOR_PIN_A, LOW);
    digitalWrite(SAG_MOTOR_PIN_B, LOW);
  }
}*/
