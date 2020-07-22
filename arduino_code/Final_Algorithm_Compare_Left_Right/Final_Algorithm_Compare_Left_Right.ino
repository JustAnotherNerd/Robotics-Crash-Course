// Include necessary libraries (PID, Motor Control, Sensor, Accelerometer)
#include <MPU6050.h>
#include <PID_v1.h>
#include <motor_control.h>
#include <HC_SR04.h>
#include <Servo.h>

// Create MPU6050 and Servo objects
MPU6050 IMU(4, 5);
Servo actuator;

// Define global variables for Sensor and PID
#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0
#define DESIRED_DISTANCE 20
#define WAIT 100

int distance;
int left_distance, right_distance;
double set_ang_vel = 0;
double input;
double output;
double kp = 2;
double ki = 0.15;
double kd = 0.01;

// Create sensor and PID objects
PID controller(&input, &output, &set_ang_vel, kp, ki, kd, REVERSE);
HC_SR04 dist_sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);

void setup() {
  // Initialize motor
  motor_setup();

  // Initialize servo
  actuator.attach(9);
  actuator.write(90);

  // Initialize MPU6050
  IMU.initialize();
  IMU.calibrate();

  // Initialize PID object
  controller.SetOutputLimits(-105, 105);
  controller.SetSampleTime(25);
  controller.SetMode(1);

  // Initalize sensor object
  dist_sensor.begin();
  dist_sensor.start();

  // Serial.begin for debugging
  Serial.begin(9600);
}
void loop(){
  // Find distance to object in front
  delay(50);
  if(dist_sensor.isFinished()) {
    distance = dist_sensor.getRange();
    dist_sensor.start();
  }
  Serial.print("Distance: ");
  Serial.println(distance);

  // Continue driving if far enough away (>20 cm away)
  if(distance > DESIRED_DISTANCE){
    drive();
  }

  // Check distances to left and right
  else{
    // Turn sensor to left, check left distance
    actuator.write(180);
    delay(WAIT);
    if(dist_sensor.isFinished()) {
      left_distance = dist_sensor.getRange();
      dist_sensor.start();
    }
    Serial.print("Left Distance: ");
    Serial.println(left_distance);
    // Turn sensor to right, wait for sensor, then check right distance
    actuator.write(0);
    delay(2 * WAIT);
    if(dist_sensor.isFinished()) {
      right_distance = dist_sensor.getRange();
      dist_sensor.start();
    }
    Serial.print("Right Distance: ");
    Serial.println(right_distance);
    actuator.write(90);
    delay(WAIT);
    // Compare distances, and turn towards the direction with greatest distance [default to right]
    if(left_distance > right_distance){
      raw_motor_control(255, -255);
    }
    else{
      raw_motor_control(-255, 255);
    }
    delay(WAIT);
  }
}

void drive() {
  IMU.update();
  input = IMU.get_ang_vel('z');

  controller.Compute();
  Serial.print("Output: ");
  Serial.println(output);

  raw_motor_control(150 + output, 150 - output);
}
