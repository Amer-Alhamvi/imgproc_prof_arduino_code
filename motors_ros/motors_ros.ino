////////////////
// ROS SETUP
////////////////
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;

//#define USE_USBCON

#include <ros.h>
#include <ros/time.h>
#include <tiny_msgs/tinyVector.h>
#include <tiny_msgs/tinyIMU.h>
ros::NodeHandle  nh;

tiny_msgs::tinyIMU imu_msg;
ros::Publisher imu_pub("tinyImu", &imu_msg);

uint32_t seq;

////////////////
// ROS SETUP
////////////////
#include <AFMotor.h>
char input;

AF_DCMotor motorL1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor motorL2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motorR1(3, MOTOR34_64KHZ); // create motor #3, 64KHz pwm
AF_DCMotor motorR2(4, MOTOR34_64KHZ); // create motor #4, 64KHz pwm

void setup()
{
  ////////////////
  // ROS SETUP
  ////////////////
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  nh.initNode();
  nh.advertise(imu_pub);
  
  accelgyro.initialize();
  seq = 0;

  ////////////////
  // MOTORS SETUP
  ////////////////
  // Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  
  motorL1.setSpeed(255);     // set the speed to 200/255
  motorL2.setSpeed(255);     // set the speed to 200/255
  motorR1.setSpeed(255);     // set the speed to 200/255
  motorR2.setSpeed(255);     // set the speed to 200/255  

}

void loop()
{
  seq++;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = 0;
  imu_msg.header.seq = seq;

  imu_msg.accel.x = ax;
  imu_msg.accel.y = ay;
  imu_msg.accel.z = az;
  imu_msg.gyro.x = gx;
  imu_msg.gyro.y = gy;
  imu_msg.gyro.z = gz;

  imu_pub.publish( &imu_msg );
  nh.spinOnce();

  if(Serial.available()){
    input = Serial.read();
    Serial.println("here");
  
    Serial.println(input);
    switch (input){
      
      case 'w':
      motorL1.run(FORWARD);
      motorL2.run(FORWARD);
      motorR1.run(FORWARD);
      motorR2.run(FORWARD);
      break;
      
      case 's':
      motorL1.run(BACKWARD);
      motorL2.run(BACKWARD);
      motorR1.run(BACKWARD);
      motorR2.run(BACKWARD);
      break;
      
      case 'a':
      motorL1.run(FORWARD);
      motorL2.run(FORWARD);
      motorR1.run(BACKWARD);
      motorR2.run(BACKWARD);
      break;
      
      case 'd':
      motorL1.run(BACKWARD);
      motorL2.run(BACKWARD);
      motorR1.run(FORWARD);
      motorR2.run(FORWARD);
      break;

      default:
      Serial.println("at defult now");
      motorL1.run(FORWARD);
      motorL2.run(FORWARD);
      motorR1.run(FORWARD);
      motorR2.run(FORWARD);
      /*motorL1.run(RELEASE);
      motorL2.run(RELEASE);
      motorR1.run(RELEASE);
      motorR2.run(RELEASE);*/
      break;
      
    }
  } else {
      motorL1.run(FORWARD);
      motorL2.run(FORWARD);
      motorR1.run(FORWARD);
      motorR2.run(FORWARD);
  }
  
}
