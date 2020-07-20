#include <HC_SR04.h>
#include <PID_v1.h>
#include <motor_control.h>
#include <Servo.h>

// Global Variables

#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0

HC_SR04 sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);
Servo actuator;

double setpoint = 10.0;
double input;
double output;
double kp = 40;
double ki = 0.4;
double kd = 10;
double scale_factor = 1.0875;

PID controller(&input, &output, &setpoint, kp, ki, kd, REVERSE);

void setup() {
  motor_setup();

  actuator.attach(9);
  actuator.write(90);

  sensor.begin();
  sensor.start();

  controller.SetOutputLimits(-255, 255);
  controller.SetSampleTime(25);
  controller.SetMode(1);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(sensor.isFinished()){
    input = sensor.getRange();
    sensor.start();
  }

  controller.Compute();

  raw_motor_control(scale_factor*output, output);
  Serial.print("Distance: ");
  Serial.println(input);
  Serial.print("Output: ");
  Serial.println(output);
}
