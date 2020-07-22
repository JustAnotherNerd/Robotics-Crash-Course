#include <MPU6050.h>
#include <PID_v1.h>
#include <motor_control.h>
#include <Servo.h>

MPU6050 IMU(4, 5);
Servo actuator;

double set_ang_vel = 0;
double input;
double output;
double kp = 2;
double ki = 0.15;
double kd = 0.01;

PID controller(&input, &output, &set_ang_vel, kp, ki, kd, REVERSE);

void setup() {
  // Initialize the components (Motors, servo, sensor, controller) and states
  motor_setup();

  actuator.attach(9);
  actuator.write(90);

  IMU.initialize();
  IMU.calibrate();

  // controller_state.setLinearState(150);
  // controller_state.setRotationState(0);

  controller.SetOutputLimits(-105, 105);
  controller.SetSampleTime(25);
  controller.SetMode(1);

  Serial.begin(9600);
}

void loop() {
  IMU.update();

  Serial.print("Ang. Vel. z: ");
  input = IMU.get_ang_vel('z');
  Serial.println(input);

  controller.Compute();
  Serial.print("Output: ");
  Serial.println(output);

  raw_motor_control(150 + output, 150 - output);

}
