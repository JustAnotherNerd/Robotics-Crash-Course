# include <Servo.h>
# include <MPU6050.h>

MPU6050 IMU(4,5);
Servo s1;

float servoPos = 90;
float theoreticalServoPos = 90;

void setup() {
  // Initializes and calibrates the MPU6050
  IMU.initialize();
  IMU.calibrate();

  // Set inital servo position
  s1.attach(9);
  s1.write(90);
  
  Serial.begin(9600);
}

void loop() {
  // Gets new readings from sensor
  IMU.update();

  // Replace this with other functions to get different information from the sensor
  Serial.print("Accel. x: ");
  float accel_x = IMU.get_accel('x');
  Serial.println(accel_x);

  theoreticalServoPos = 90 + 90 * accel_x;
  Serial.print("Estimated Servo Position: ");
  Serial.println(theoreticalServoPos);
  servoPos = theoreticalServoPos;
  if(servoPos > 180){servoPos = 180;}
  if(servoPos < 0){servoPos = 0;}
  
  s1.write(servoPos);
}
