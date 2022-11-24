#include <webots/Robot.hpp>

// Added a new include file
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include "PID.h"
#define TIME_STEP 8

#define MAX_SPEED 0.628

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
 Robot *robot = new Robot();

 // get a handler to the motors and set target position to infinity (speed control)
 Motor *leftMotor = robot->getMotor("motorLeft");
 Motor *rightMotor = robot->getMotor("motorRight");
 leftMotor->setPosition(INFINITY);
 rightMotor->setPosition(INFINITY);
 leftMotor->setVelocity(0);
 rightMotor->setVelocity(0);
 InertialUnit *inertial = robot->getInertialUnit("inertial");
 inertial->enable(TIME_STEP);

 // set up the motor speeds at 10% of the MAX_SPEED.
    double j = 0; //KP*j
    double k = 0;
    double m = 0;
    double SUM = 0;
    double diff = 0;
    double KP = 34;
    double KI = 16;
    double KD = 5;
    double vel = 0.00;


 while (robot->step(TIME_STEP) != -1){
     printf("\n");

     leftMotor->setVelocity(vel);
     rightMotor->setVelocity(vel);
     diff = inertial->getRollPitchYaw()[0] - m;
     m =  inertial->getRollPitchYaw()[0];
     j =  inertial->getRollPitchYaw()[0] - (0) * 1.5708 / 180;
     k = j;
     SUM = SUM + k;
     printf("j: %lf ", j);

     printf("sum: %lf ",SUM);
     vel = (j * KP + SUM * KI + diff * KD);
     printf("vel: %lf ",vel);
     if (vel == 0)
     {
         vel = 0.20;
     }
     if (vel == 15)
     {
         vel = 14.8;
     }
 };

 delete robot;

 return 0;
}