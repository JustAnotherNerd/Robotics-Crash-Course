// Import libraries 

#include <PID_v1.h> 

#include <motor_control.h> 

  

#include <HC_SR04.h> 

#define TRIG_PIN 10 

#define ECHO_PIN 2 

#define ECHO_INT 0 

HC_SR04 dist_sensor(TRIG_PIN, ECHO_PIN, ECHO_INT); 

  

#include <State.h> 

State controller_state; 

  

#include <MPU6050.h> 

#define SDA 4 

#define SCL 5 

MPU6050 sensor(SDA, SCL); 

  

int base_motor_speed = 150; 

double setpoint = 0; 

double input; 

double output; 

double kp = 2; 

double ki = 0; 

double kd = 0.5; 

PID controller(&input, &output, &setpoint, kp, ki, kd, REVERSE); 

  

// Initializing arrays 

const int len = 18; 

int rot_arr[len] = {}; 

int dist_arr[len] = {}; 

  

void setup() { 

  // Setup motors 

  motor_setup(); 

  

  // Initialize sensor 

  sensor.initialize(); 

  sensor.update(); 

  

  // Initialize distance sensor 

  dist_sensor.begin(); 

  dist_sensor.start(); 

  

  // Set PID parameters 

  controller.SetOutputLimits(-100, 100); 

  controller.SetSampleTime(50); 

  controller.SetMode(1); 

  

  // Initialize states 

  controller_state.setLinearState(150); 

  controller_state.setRotationState(0); 

  

  // Begin serial for debugging 

  Serial.begin(9600); 

} 

  

void loop() { 

  // Sets angular velocity to input in PID 

  sensor.update(); 

  double angVelReading = sensor.get_ang_vel('z'); 

  input = angVelReading; 

  

  for (int i = 1; i <= 18; i++) { 

    // Stops the linear state of car 

    controller_state.setLinearState(10); 

  

    // Allows the robot to rotate in 20 degree intervals 

    int rot_val = i * 20; 

    controller_state.setRotationState(rot_val); 

    delay(1); 

    double setpoint = controller_state.getRotationState(); 

    rot_arr[i] = rot_val; 

  

    controller.Compute(); 

  

    raw_motor_control(controller_state.getLinearState() - output, controller_state.getLinearState() + output); 

  

    // Allows distance sensor to record 

    if (dist_sensor.isFinished()) { 

      delay(1); 

      int val = dist_sensor.getRange(); 

      dist_sensor.start(); 

      dist_arr[i] = val; 

    } 

  } 

  

  controller_state.setLinearState(0); 

  // Finds the i value with the largest distance 

  int direct = findInt(dist_arr, len); 

  // Sets the car to rotate to direction with largest distance 

  controller_state.setRotationState(rot_arr[direct]); 

  double setpoint = controller_state.getRotationState(); 

  // Resets linear state back to 150 

  controller_state.setLinearState(150); 

  controller_state.setRotationState(0); 

  

  controller.Compute(); 

  

  // Set motor power from PID 

  raw_motor_control(controller_state.getLinearState() - output, controller_state.getLinearState() + output); 

  

  Serial.println(direct); 

} 

  

// Finds max interger value matching max distance 

int findInt(int arr[], int len) { 

  int maxVal = arr[0]; 

  int maxInt = 0; 

  for (int i = 1; i < len; i++) { 

    if (arr[i] > maxVal) { 

      delay(50); 

      maxVal = arr[i]; 

      maxInt = i; 

    } 

    return maxInt; 

  } 

}  
