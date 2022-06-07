// (c) Michael Schoeffler 2017, http://www.mschoeffler.de

#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <AFMotor.h> // This library is for controlling the motors

char motors_input; // Input from PC to conotrol motors

AF_DCMotor motorL1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor motorL2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motorR1(3, MOTOR34_64KHZ); // create motor #3, 64KHz pwm
AF_DCMotor motorR2(4, MOTOR34_64KHZ); // create motor #4, 64KHz pwm

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600);

  // Motors
  motorL1.setSpeed(255);     // set the speed of MotorL1 to 255
  motorL2.setSpeed(255);     // set the speed of MotorL2 to 255
  motorR1.setSpeed(255);     // set the speed of MotorR1 to 255
  motorR2.setSpeed(255);     // set the speed of MotorR2 to 255
  
  // wire
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
void loop() {
  /////////////////////////////
  // Motors
  /////////////////////////////
  if(Serial.available()){
    motors_input = Serial.read();
  
    switch (motors_input){
      
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

      case 'x':
      motorL1.run(RELEASE);
      motorL2.run(RELEASE);
      motorR1.run(RELEASE);
      motorR2.run(RELEASE);
      break;

      default:
      /*motorL1.run(RELEASE);
      motorL2.run(RELEASE);
      motorR1.run(RELEASE);
      motorR2.run(RELEASE);*/      
      break;
      
    }
  } else {
      motorL1.run(RELEASE);
      motorL2.run(RELEASE);
      motorR1.run(RELEASE);
      motorR2.run(RELEASE);
  }

  /////////////////////////////
  // IMU
  /////////////////////////////
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  
  // delay
  delay(1000);
}
