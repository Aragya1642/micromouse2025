#include <Arduino.h>
#include <pins.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include "LedDisplay.h"

#include "MPU9250.h"

#include <motor.h>
#include <sensor.h>

#include <string>

#include "chassis.h"
#include "maze.h"

Encoder frontRightEnc(ENCODER_FRONT_RIGHT_B, ENCODER_FRONT_RIGHT_A);
Encoder backRightEnc(ENCODER_BACK_RIGHT_A, ENCODER_BACK_RIGHT_B);
Encoder frontLeftEnc(ENCODER_FRONT_LEFT_B, ENCODER_FRONT_LEFT_A);
Encoder backLeftEnc(ENCODER_BACK_LEFT_B , ENCODER_BACK_LEFT_A);

MPU9250 mpu;


Sensor leftSideIR;
Sensor rightSideIR;

Sensor leftAngleIR;
Sensor rightAngleIR;

Sensor leftFrontIR;
Sensor rightFrontIR;

Motor frontLeftMotor;
Motor frontRightMotor;
Motor backLeftMotor;
Motor backRightMotor;

MiniPID frontLeftPID(MOTOR_VEL_PID_P, MOTOR_VEL_PID_I, MOTOR_VEL_PID_D, MOTOR_VEL_PID_F);
MiniPID frontRightPID(MOTOR_VEL_PID_P, MOTOR_VEL_PID_I, MOTOR_VEL_PID_D, MOTOR_VEL_PID_F);
MiniPID backLeftPID(MOTOR_VEL_PID_P, MOTOR_VEL_PID_I, MOTOR_VEL_PID_D, MOTOR_VEL_PID_F);
MiniPID backRightPID(MOTOR_VEL_PID_P, MOTOR_VEL_PID_I, MOTOR_VEL_PID_D, MOTOR_VEL_PID_F);

Chassis chassis;

MiniPID anglePID(0,000001,0,0);
MiniPID turnPID(0.3,0.001,0);
MiniPID distancePID(0.0001,0.00000001,0);


Location m_location;
Heading m_heading;

Maze maze; 

void setup() {
    Wire1.setClock(400000);
    Serial.begin(115200);
    Wire1.begin();

    mpu.setup(0x68, MPU9250Setting(), Wire1);
    distancePID.setOutputRampRate(0.01);
    /*delay(2000);
    Serial.print("accel setup");
    mpu.calibrateAccelGyro();
    Serial.print("mag setup ");
    delay(1000);
    mpu.calibrateMag();*/

  //   pinMode(BUTTON_1, INPUT);
  //   pinMode(BUTTON_2, INPUT);

    pinMode(MOTOR_LEFT_IN_1, OUTPUT);
    pinMode(MOTOR_LEFT_IN_2, OUTPUT);
    pinMode(MOTOR_BACK_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_FRONT_LEFT_PWM, OUTPUT);

    pinMode(MOTOR_RIGHT_IN_1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN_2, OUTPUT);
    pinMode(MOTOR_BACK_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_FRONT_RIGHT_PWM, OUTPUT);

    frontLeftMotor.setMotorPins(MOTOR_LEFT_IN_1, MOTOR_LEFT_IN_2, MOTOR_FRONT_LEFT_PWM);
    frontLeftMotor.setMotorAttr(MAX_VELOCITY);
    frontLeftMotor.setEncoder(&frontLeftEnc);
    frontLeftMotor.setVelPID(&frontLeftPID);
    frontLeftMotor.initMotor();

    backLeftMotor.setMotorPins(MOTOR_LEFT_IN_1, MOTOR_LEFT_IN_2, MOTOR_BACK_LEFT_PWM);
    backLeftMotor.setMotorAttr(MAX_VELOCITY);
    backLeftMotor.setEncoder(&backLeftEnc);
    backLeftMotor.setVelPID(&backLeftPID);
    backLeftMotor.initMotor();

    frontRightMotor.setMotorPins(MOTOR_RIGHT_IN_1, MOTOR_RIGHT_IN_2, MOTOR_FRONT_RIGHT_PWM);
    frontRightMotor.setMotorAttr(MAX_VELOCITY);
    frontRightMotor.setEncoder(&frontRightEnc);
    frontRightMotor.setVelPID(&frontRightPID);
    frontRightMotor.initMotor();

    backRightMotor.setMotorPins(MOTOR_RIGHT_IN_1, MOTOR_RIGHT_IN_2, MOTOR_BACK_RIGHT_PWM);
    backRightMotor.setMotorAttr(MAX_VELOCITY);
    backRightMotor.setEncoder(&backRightEnc);
    backRightMotor.setVelPID(&backRightPID);
    backRightMotor.initMotor();

    //pinMode(BUZZER, OUTPUT);

    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);


    leftSideIR.setSensorPins(EMITTER_LEFT_HALF, IR_LEFT_SIDE);
    leftSideIR.initSensor();
    leftAngleIR.setSensorPins(EMITTER_LEFT_HALF, IR_LEFT_ANGLE);
    leftAngleIR.initSensor();
    leftFrontIR.setSensorPins(EMITTER_LEFT_HALF, IR_LEFT_FRONT);
    leftFrontIR.initSensor();
    rightSideIR.setSensorPins(EMITTER_RIGHT_HALF, IR_RIGHT_SIDE);
    rightSideIR.initSensor();
    rightAngleIR.setSensorPins(EMITTER_RIGHT_HALF, IR_RIGHT_ANGLE);
    rightAngleIR.initSensor();
    rightFrontIR.setSensorPins(EMITTER_RIGHT_HALF, IR_RIGHT_FRONT);
    rightFrontIR.initSensor();

    chassis.setMotors(&backRightMotor, &backLeftMotor, &frontRightMotor, &frontLeftMotor);
    chassis.setChassisAttr(WHEEL_DIAMETER, ENCODER_TICKS_PER_WHEEL_ROTATION, WHEEL_TRACK);
    chassis.setPID(&distancePID, &anglePID, &turnPID);
    //chassis.setMPU(&mpu);
    chassis.setError(7, 2);
  
}


void update_map() {
  digitalWrite(EMITTER_LEFT_HALF, HIGH);
  digitalWrite(EMITTER_RIGHT_HALF, HIGH);
  bool leftWall = analogRead(IR_LEFT_ANGLE) >= 200;
  bool frontWall = analogRead(IR_LEFT_FRONT) >= 200;
  bool rightWall = analogRead(IR_RIGHT_ANGLE) >= 200;
  char w[] = "--- ";
  if (leftWall) {
    w[0] = 'L';
  };
  if (frontWall) {
    w[1] = 'F';
  };
  if (rightWall) {
    w[2] = 'R';
  };
  Serial.print(w);
  switch (m_heading) {
    case NORTH:
      maze.update_wall_state(m_location, NORTH, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, EAST, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, WEST, leftWall ? WALL : EXIT);
      break;
    case EAST:
      maze.update_wall_state(m_location, EAST, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, SOUTH, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, NORTH, leftWall ? WALL : EXIT);
      break;
    case SOUTH:
      maze.update_wall_state(m_location, SOUTH, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, WEST, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, EAST, leftWall ? WALL : EXIT);
      break;
    case WEST:
      maze.update_wall_state(m_location, WEST, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, NORTH, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, SOUTH, leftWall ? WALL : EXIT);
      break;
    default:
      // This is an error. We should handle it.
      break;
  }
}

void move_ahead(){
  chassis.moveForwardTile();
}

void turn_right(){
  chassis.moveForwardTile();
  chassis.turnRight();
  m_heading = right_from(m_heading);
}

void turn_back(){
  chassis.turnRight();
  chassis.turnRight();
  chassis.moveForwardTile();
  m_heading = behind_from(m_heading);
}

void turn_left(){

  chassis.moveForwardTile();
  chassis.turnLeft();
  m_heading = left_from(m_heading);
}

void search_maze(){
  m_location = START;
  m_heading = NORTH;
  maze.initialise();
  maze.set_goal(Location(7,7));

  maze.flood(maze.goal());
  while (m_location != maze.goal()){
    m_location = m_location.neighbour(m_heading);
    update_map();
    maze.flood(maze.goal());
    unsigned char newHeading = maze.heading_to_smallest(m_location, m_heading);
    unsigned char hdgChange = (newHeading - m_heading) & 0x3;
    if (m_location != maze.goal()) {
      switch (hdgChange) {
        // each of the following actions will finish with the
        // robot moving and at the sensing point ready for the
        // next loop iteration
        case AHEAD:
          move_ahead();
          break;
        case RIGHT:
          turn_right();
          break;
        case BACK:
          turn_back();
          break;
        case LEFT:
          turn_left();
          break;
      }
    }
  }
}


void loop() {

  digitalWrite(EMITTER_LEFT_HALF, HIGH);
  while(1000>analogRead(IR_LEFT_FRONT)){
    delay(10);
  }
  Serial.printf("begin\n");
  delay(1000);
  search_maze();
  //turn_left();


    
    /*Serial.printf("IMU test\n");
    while(true){
        //IMU.getGyro(&gyroData);
        // auto result = mySensor.magUpdate();
        // if (result != 0) {
        //   mySensor.beginMag();
        // result = mySensor.magUpdate();
        // }
        // mySensor.magUpdate();
        //  mySensor.gyroUpdate();
        // mySensor.accelUpdate();
        
        // Serial.printf("Mag X:%f Y:%f Z:%f\n", mySensor.magX(), mySensor.magY(), mySensor.magZ());
        // Serial.printf("Gyro X:%f Y:%f Z:%f\n", mySensor.gyroX(), mySensor.gyroY(), mySensor.gyroZ());
        // Serial.printf("Accel X:%f Y:%f Z:%f\n", mySensor.accelX(), mySensor.accelY(), mySensor.accelZ());
        if (mpu.update()) {
          Serial.print(mpu.getYaw()); Serial.print(", ");
          Serial.print(mpu.getPitch()); Serial.print(", ");
          Serial.println(mpu.getRoll()); Serial.print("\n");
      }*/

      /*if (mpu.update()) {
        Serial.print(mpu.getYaw()); Serial.print(", ");
        Serial.print(mpu.getPitch()); Serial.print(", ");
        Serial.println(mpu.getRoll()); Serial.print("\n");
      }*/

      /*digitalWrite(EMITTER_LEFT_HALF, HIGH);
      while(1000>analogRead(IR_LEFT_FRONT)){
        delay(10);
      }

      delay(500);
      chassis.moveForwardTile();
     
      chassis.turnRight();

      chassis.moveForwardTile();
      chassis.moveForwardTile();

      chassis.turnRight();

      chassis.moveForwardTile();

      chassis.turnLeft();

      chassis.moveForwardTile();

      Serial.printf("help\n");*/
      
        

}

