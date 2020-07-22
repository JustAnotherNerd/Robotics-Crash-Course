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
#define SERVO_PIN 9
#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0
#define CENTER 90
#define DESIRED_DISTANCE 30

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
  if(dist_sensor.isFinished()) {
    distance = dist_sensor.getRange();
    dist_sensor.start();
  }

  // Continue driving if far enough away (>20 cm away)
  if(distance > 20){
    drive();
  }

  // Check distances to left and right
  else{
    // Turn sensor to left, check left distance
    actuator.write(180);
    if(dist_sensor.isFinished()) {
      left_distance = dist_sensor.getRange();
      dist_sensor.start();
    }
    // Turn sensor to right, wait for sensor, then check right distance
    actuator.write(0);
    delay(50);
    if(dist_sensor.isFinished()) {
      right_distance = dist_sensor.getRange();
      dist_sensor.start();
    }
    // Compare distances, and turn towards the direction with greatest distance [default to right]
    if(left_distance > right_distance){
      
    }
    else{
      
    }
  }
}

void drive() {
  IMU.update();
  
  Serial.print("Ang. Vel. z: ");
  input = IMU.get_ang_vel('z');
  Serial.println(input);

  controller.Compute();
  Serial.print("Output: ");
  Serial.println(output);

  raw_motor_control(150 + output, 150 - output);
}
