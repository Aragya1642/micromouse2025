#include <cmath>
#include <tuple>
#include <numbers>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <utility>
#include <cstdint>
#include <math.h>
#include <chassis.h>

Chassis::Chassis(){
    currentPos.x = 0;
    currentPos.y = 0;
    currentPos.rotation = 0;

    lastLeftEnc = 0;
    lastRightEnc = 0;
}
    
void Chassis::setChassisAttr(float ws, float er, float wt){
    wheelDiameter = ws;
    encRatio = er;
    wheelTrack = wt;
}
void Chassis::setMotors(Motor *brm, Motor *blm, Motor *frm, Motor *flm){
    backRightMotor = brm;
    frontRightMotor = frm;
    backLeftMotor = blm;
    frontLeftMotor = flm;
}
void Chassis::setMPU(MPU9250 *imu){
    mpu = imu;
}
void Chassis::setPID(MiniPID *dPID, MiniPID *aPID, MiniPID *tPID){
    distancePID = dPID;
    anglePID = aPID;
    turnPID = tPID;
}

void Chassis::setError(float dError, float aError){

    distanceError = dError;
    angleError =  aError ;
}
double Chassis::getLeftEnc(){
    return ((double) frontLeftMotor->getEncoder() + (double) backLeftMotor->getEncoder()) / 2; 
}

double Chassis::getRightEnc(){
    return ((double) frontRightMotor->getEncoder() + (double) backRightMotor->getEncoder()) / 2; 
}


void Chassis::updatePosition() {

    int leftEncDelta = getLeftEnc() - lastLeftEnc;
    int rightEncDelta = getRightEnc() - lastRightEnc;

    float orientation = currentPos.rotation;

    double tickToMM = M_PI * wheelDiameter / encRatio; 
    double lDis = leftEncDelta * tickToMM;
    double rDis = rightEncDelta * tickToMM;
    //calculate the change in orientation
    double deltaTheta = (lDis- rDis) / wheelTrack; //in radians
    //local cordinates for the movement
    //y is the direction of the movement
    //x is perpendicular
    double localOffsetY;
    if (deltaTheta != 0) {
        //if movement was an arc
        //calculate y
        localOffsetY = 2 * std::sin(deltaTheta / 2) * (rDis / deltaTheta + wheelTrack / 2);
    } 
    else {
        //if each wheel spun the same amount
        localOffsetY = rDis;
    }
    //convert local to polar cords and then to field cords
    double avgA = orientation + (deltaTheta / 2);
    
    double polarR = std::sqrt(localOffsetY * localOffsetY);
    double polarA = std::atan2(localOffsetY, 0) - avgA;
    double dX = std::sin(polarA) * polarR;
    double dY = std::cos(polarA) * polarR;
    if (isnan(dX)){
        dX = 0;
    }
    if (isnan(dY)) {
        dY = 0;
    }
    if (isnan(deltaTheta)) {
        deltaTheta = 0;
    }
    //update instance varibles
    currentPos.x += dX;
    currentPos.y += dY;
    currentPos.rotation += deltaTheta;

    lastLeftEnc = getLeftEnc();
    lastRightEnc = getRightEnc();

}
//blocking code with no async 
//if future implementation requires a step method can be implemented
void Chassis::moveForwardTile(){
    frontLeftMotor->resetEncoder();
    frontRightMotor->resetEncoder();
    backLeftMotor->resetEncoder();
    backRightMotor->resetEncoder();
    do{
        driveVector(distancePID->getOutput((getLeftEnc()+getRightEnc())/2, 180 / (M_PI * wheelDiameter) * 360),0);
        Serial.printf("last error: %f cur pos: %f target: %f\n",distancePID->getLastError(), (getLeftEnc()+getRightEnc())/2, 180 / (M_PI * wheelDiameter) * 360);
    }while( abs(distancePID->getLastError()) >= distanceError);

    frontLeftMotor->stop();
    frontRightMotor->stop();
    backLeftMotor->stop();
    backRightMotor->stop();

    distancePID->reset();
    
}

void Chassis::turnLeft(){


    frontLeftMotor->resetEncoder();
    frontRightMotor->resetEncoder();
    backLeftMotor->resetEncoder();
    backRightMotor->resetEncoder();
    do{
        driveVector(0, -distancePID->getOutput(getRightEnc(), 75 / (M_PI * wheelDiameter) * 360));
        Serial.printf("last error: %f cur pos: %f target: %f\n",distancePID->getLastError(), (getLeftEnc()+getRightEnc())/2, 180 / (M_PI * wheelDiameter) * 360);
    }while( abs(distancePID->getLastError()) >= distanceError);

    frontLeftMotor->stop();
    frontRightMotor->stop();
    backLeftMotor->stop();
    backRightMotor->stop();

    distancePID->reset();
    
}

void Chassis::turnRight(){
    frontLeftMotor->resetEncoder();
    frontRightMotor->resetEncoder();
    backLeftMotor->resetEncoder();
    backRightMotor->resetEncoder();
    do{
        driveVector(0, distancePID->getOutput(getLeftEnc(), 75 / (M_PI * wheelDiameter) * 360));
        Serial.printf("last error: %f cur pos: %f target: %f\n",distancePID->getLastError(), (getLeftEnc()+getRightEnc())/2, 180 / (M_PI * wheelDiameter) * 360);
    }while( abs(distancePID->getLastError()) >= distanceError);

    frontLeftMotor->stop();
    frontRightMotor->stop();
    backLeftMotor->stop();
    backRightMotor->stop();

    distancePID->reset();
    
}
void Chassis::moveToPostion(double x, double y){
    currentPos.rotation = 0;
    currentPos.x = 0;
    currentPos.y = 0;

    position target = position{x, y, 0};
    target.rotation = pointAngle(currentPos, target);

    distancePID->setSetpoint(0);

    do{
        
        updatePosition();

        driveVector(
            -distancePID->getOutput(pointDistance(currentPos,target), 0) , 
            0);
        Serial.printf("last error: %f cur pos: %f target: %f\n", distancePID->getLastError(), pointDistance(currentPos,target), 0);

    } while( abs(distancePID->getLastError()) >= distanceError);

    frontLeftMotor->stop();
    frontRightMotor->stop();
    backLeftMotor->stop();
    backRightMotor->stop();

    distancePID->reset();
    anglePID->reset();
}

double Chassis::pointDistance(position pos1, position pos2){
    return sqrt(pow(pos2.x - pos1.x,2) + pow(pos2.y - pos1.y,2));
}

double Chassis::pointAngle(position pos1, position pos2){
    return atan((pos2.y - pos1.y)/(pos2.x-pos1.x));
}

void Chassis::turnToOrientation(double theta){
    turnTargetTime = 0;
    theta = theta * M_PI/180;
    /*while(abs(anglePID->getLastError()) >= angleError){
        Serial.print("here\n");
        updatePosition();
        driveVector(0, turnPID->getOutput(currentPos.rotation, theta));
        delayMicroseconds(5000);
    }*/
    do{
        //printPosition();
        updatePosition();
        //driveVector(0, turnPID->getOutput(currentPos.rotation, theta));
        Serial.printf("last error: %f cur rot: %f target: %f\n", turnPID->getLastError(), currentPos.rotation, theta);
        delayMicroseconds(5000);
    }
    while(!turnIsSettled());

    turnPID->reset();
}

void Chassis::turnOrientation(double theta){
    currentPos.rotation = 0;
    currentPos.x = 0;
    currentPos.y = 0;
    turnTargetTime = 0;
    theta = theta * M_PI/180;
    /*while(abs(anglePID->getLastError()) >= angleError){
        Serial.print("here\n");
        updatePosition();
        driveVector(0, turnPID->getOutput(currentPos.rotation, theta));
        delayMicroseconds(5000);
    }*/
    do{
        //printPosition();
        updatePosition();
        driveVector(0, turnPID->getOutput(currentPos.rotation, theta));
        Serial.printf("last error: %f cur rot: %f target: %f\n", turnPID->getLastError(), currentPos.rotation, theta);
        delayMicroseconds(5000);
    }
    while(!turnIsSettled());

    frontLeftMotor->stop();
    frontRightMotor->stop();
    backLeftMotor->stop();
    backRightMotor->stop();

    turnPID->reset();
}


float wrapAngle(float angle) {
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
}

void Chassis::gyroTurnOrientation(double theta){
    turnTargetTime = 0;
    mpu->update();
    double targetTheta = wrapAngle(mpu->getYaw() + theta);
    /*while(abs(anglePID->getLastError()) >= angleError){
        Serial.print("here\n");
        updatePosition();
        driveVector(0, turnPID->getOutput(currentPos.rotation, theta));
        delayMicroseconds(5000);
    }*/
    do{
        //printPosition();
        //updatePosition();

        //Serial.printf("here4");
        if(mpu->update()){
            driveVector(0, turnPID->getOutput(mpu->getYaw(), targetTheta));
            Serial.printf("last error: %f cur rot: %f target: %f\n", turnPID->getLastError(), mpu->getYaw(), targetTheta);
            delayMicroseconds(5000);
        }
    }
    while(!turnIsSettled());
    turnPID->reset();
}

//set drive train to follow a vector
void Chassis::driveVector(double velocity, double theta){
    //set max velocities between -1 and 1
    const double forwardSpeed = std::clamp(velocity, -1.0, 1.0);
    const double yaw = std::clamp(theta, -1.0, 1.0);
    //Serial.printf("yaw: %f\n", yaw);
    //turn motors with yaw
    double leftOutput = forwardSpeed + yaw;
    double rightOutput = forwardSpeed - yaw;
    //continue turning if one of the motor is over max velocity
    if (const double maxInputMag = std::max<double>(std::abs(leftOutput), std::abs(rightOutput)); maxInputMag > 1) {
        leftOutput /= maxInputMag;
        rightOutput /= maxInputMag;

    } 
    //Serial.printf("left output: %f right output: %f\n", leftOutput, rightOutput);
    frontLeftMotor->setVelocity(leftOutput);
    frontLeftMotor->stepVelocityPID();
    backLeftMotor->setVelocity(leftOutput);
    backLeftMotor->stepVelocityPID();

    frontRightMotor->setVelocity(rightOutput);
    frontRightMotor->stepVelocityPID();
    backRightMotor->setVelocity(rightOutput);
    backRightMotor->stepVelocityPID();
    
}

bool Chassis::turnIsSettled(){
    
    if(abs(turnPID->getLastError()) >= angleError){
        turnTargetTime = 0;
    }
    return (500000 <= turnTargetTime) && (abs(turnPID->getLastError()) <= angleError * M_PI/180 );
}

void Chassis::printPosition(){
    Serial.printf("%f|%f|%f\n", currentPos.x, currentPos.y, currentPos.rotation);
}