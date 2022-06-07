#include <AFMotor.h>
char input;

AF_DCMotor motorL1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor motorL2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motorR1(3, MOTOR34_64KHZ); // create motor #3, 64KHz pwm
AF_DCMotor motorR2(4, MOTOR34_64KHZ); // create motor #4, 64KHz pwm

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  
  motorL1.setSpeed(255);     // set the speed to 200/255
  motorL2.setSpeed(255);     // set the speed to 200/255
  motorR1.setSpeed(255);     // set the speed to 200/255
  motorR2.setSpeed(255);     // set the speed to 200/255
}

void loop() {
  Serial.println("there");
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
