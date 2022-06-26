#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;

// Uncomment for Leonardo
//#define USE_USBCON

#include <ros.h>
#include <ros/time.h>
#include <tiny_msgs/tinyVector.h>
#include <tiny_msgs/tinyIMU.h>
ros::NodeHandle  nh;

tiny_msgs::tinyIMU imu_msg;
ros::Publisher imu_pub("tinyImu", &imu_msg);

uint32_t seq;

void setup()
{
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
}
