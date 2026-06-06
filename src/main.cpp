#include "Alfredo_NoU3.h"
#include "Arduino.h"
#include "MT6701.h"
#include "PestoLink-Receive.h"
#include "Wire.h"
#include "esp32-hal.h"
#include <cmath>

NoU_Motor motor1(1), motor2(2), motor3(3), motor4(4), motor5(5), motor6(6),
    intake(7);

const int ANALOG_PINS[3] = {8, 9, 7};

// MT6701 i2c_encoder;

#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

class swervepod {
public:
  double p = 0.003, d = 0.000005;
  double target = 0.0;
  double last_power = 0.0;
  double mult = 1.0;
  double last_rotation = 0.0;
  unsigned long last_time;
  double last_error = 0.0;
  double last_angle = 0.0;
  NoU_Motor &rotation, &drive;

  swervepod(NoU_Motor &rotation, NoU_Motor &drive)
      : rotation(rotation), drive(drive) {
    last_time = micros();
  }

  void update(double angle, double power) {
    if (isnan(angle)) {
      rotation.set(0.0);
      drive.set(0.0);
      return;
    }
    double dT = (micros() - last_time) / 1000000.0;
    double error = fmod(target - angle + 180.0, 360.0) - 180.0;
    if (error < -180.0) {
      error += 360.0;
    }
    mult = -1.0;
    if (error > 90.0)
      error -= 180.0;
    else if (error < -90.0)
      error += 180.0;
    else
      mult = 1.0;
    // double mult = 1.0;
    // PestoLink.printfTerminal("%.2f, %.2f", angle, error);
    double rotationRateLimit = 8.0;
    double driveRateLimit = 5.0;

    if (abs(error) > 3.0) {
      double delta_angle = angle - last_error;
      if (delta_angle > 180.0)
        delta_angle -= 360.0;
      if (delta_angle < -180.0)
        delta_angle += 360.0;
      double output = -1.0 * ((p * error) + (-d * delta_angle / dT));
      if (output > 0.01) {
        output += 0.1;
      } else if (output < -0.01) {
        output += -0.1;
      }
      last_rotation +=
          constrain(output - last_rotation, dT * -rotationRateLimit,
                    dT * rotationRateLimit);
      rotation.set(output);
    } else {
      rotation.set(0.0);
      last_rotation += constrain(0.0 - last_rotation, dT * -8.0, dT * 8.0);
    }
    double target_power = power * cos(error * PI / 180.0);
    last_power += constrain(mult * target_power - last_power,
                            dT * -driveRateLimit, dT * driveRateLimit);
    last_power = constrain(last_power, -1.0, 1.0);
    drive.set(mult * target_power);
    last_error = error;
    last_angle = angle;
    last_time = micros();
  }

  void drive_direction(double power) {
    drive.set(power * mult);
    rotation.set(0.0);
  }

  void disable() {
    drive.set(0.0);
    rotation.set(0.0);
  }
};

void setup() { // put your setup code here, to run once

  NoU3.begin();
  PestoLink.begin("ReefSON");

  // Wire.end();
  // Wire.begin(6, 5);
  // Wire.setTimeOut(10);
  // delay(1000);

  Serial.begin(9600);
  Serial.println("Hello:)");

  // tcaselect(0);
  // delayMicroseconds(50);
  // left_encoder.analogModeSet();
  // left_encoder.programmEEPROM();
  // tcaselect(1);
  // delayMicroseconds(50);
  // right_encoder.analogModeSet();
  // right_encoder.programmEEPROM();
  // tcaselect(2);
  // delayMicroseconds(50);
  // front_encoder.analogModeSet();
  // front_encoder.programmEEPROM();

  for (int i = 0; i < 3; i++) {
    pinMode(ANALOG_PINS[i], INPUT);
  }

  NoU3.calibrateIMUs();
}

swervepod front_pod(motor4, motor3);
swervepod left_pod(motor2, motor1);
swervepod right_pod(motor5, motor6);

double pod_angles[] = {0.0, 0.0, 0.0};
double imu_scalar = (5.0 * 2.0 * PI) / 28.28;
void loop() { // put your main code here, to run repeatedly
              // DRIVETRAIN
  double forward = 1 * PestoLink.getAxis(1);
  double strafe = -1 * PestoLink.getAxis(0);
  double turn = PestoLink.getAxis(2);

  for (int i = 0; i < 3; i++) {
    double angle = ((double)analogRead(ANALOG_PINS[i])) / 4096.0 * 360.0;

    switch (i) {
    case 0: {
      pod_angles[i] = -angle + 242.20;
      break;
    }
    case 1: {
      pod_angles[i] = angle - 0.73;
      break;
    }
    case 2: {
      pod_angles[i] = -angle + 272.88;
      break;
    }
    }
  }

  // Read angles from sensors with multiplexer
  // tcaselect(0);
  // delayMicroseconds(50);
  // double left_angle = left_encoder.angleRead();
  // left_angle = left_angle != 0 ? -left_angle + 242.20 : NAN;
  // tcaselect(1);
  // delayMicroseconds(50);
  // double right_angle = right_encoder.angleRead();
  // right_angle = right_angle != 0 ? right_angle - 0.73 : NAN;
  // tcaselect(2);
  // delayMicroseconds(50);
  // double front_angle = front_encoder.angleRead();
  // front_angle = front_angle != 0 ? -front_angle + 272.88 : NAN;
  // Serial.print(left_angle);
  // Serial.print(" ");
  // Serial.print(right_angle);
  // Serial.print(" ");
  // Serial.println(front_angle);

  // Velocity calculations
  double heading = (NoU3.yaw * imu_scalar);
  double vx = forward * cos(heading) - strafe * sin(heading);
  double vy = forward * sin(heading) + strafe * cos(heading);
  double omega = turn;
  right_pod.drive.setInverted(true);

  Serial.println(NoU3.yaw * imu_scalar);

  double positions[][2] = {
      {-0.5, -sqrt(3) / 2.0}, {-0.5, sqrt(3) / 2.0}, {1.0, 0.0}};
  // double angles[] = {left_angle, right_angle, front_angle};
  swervepod *pods[] = {&left_pod, &right_pod, &front_pod};
  if (PestoLink.isConnected()) {
    if (abs(vx) + abs(vy) + abs(omega) > 0.2) {
      for (int i = 0; i <= 2; i++) {
        double x = vx - positions[i][1] * omega;
        double y = vy + positions[i][0] * omega;
        double vel = sqrt(x * x + y * y);
        if (vel > 0.2) {
          double angle = atan2(y, x) * 180.0 / PI;
          if (i == 1) {
            angle *= -1.0; ////
          }
          pods[i]->target = angle;
        }
        pods[i]->update(pod_angles[i], vel * (i == 1 ? -1.0 : 1.0));
        // if (i == 2) {
        //   PestoLink.printfTerminal("%.2f, %.2f", x, y);
        // }
      }
    }
    // else if (PestoLink.buttonHeld(0)) {
    //   for (int i = 0; i <= 2; i++) {
    //     pods[i]->drive_direction(i == 1 ? -1.0 : 1.0);
    //   }
    // } else if (PestoLink.buttonHeld(3)) {
    //   for (int i = 0; i <= 2; i++) {
    //     pods[i]->drive_direction(i == 1 ? 1.0 : -1.0);
    //   }
    // }
    else {
      for (int i = 0; i <= 2; i++) {
        pods[i]->drive_direction(0.0);
      }
    }
  } else {
    front_pod.disable();
    left_pod.disable();
    right_pod.disable();
  }

  // left_pod.drive.set(1.0);
  // right_pod.drive.set(1.0);
  // front_pod.drive.set(1.0);

  PestoLink.printfTerminal(
      "%.2f||%.3f|| %.2f,%.2f,%.2f | %.2f,%.2f,%.2f | %.2f,%.2f,%.2f",
      NoU3.getBatteryVoltage(), NoU3.yaw * imu_scalar, left_pod.target,
      pod_angles[0], left_pod.last_error, right_pod.target, pod_angles[1],
      right_pod.last_error, front_pod.target, pod_angles[2],
      front_pod.last_error);

  if (PestoLink.buttonHeld(9)) {
    front_pod.disable();
    left_pod.disable();
    right_pod.disable();
    NoU3.calibrateIMUs();
    delay(500);
  }
}