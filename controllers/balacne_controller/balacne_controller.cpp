#include <webots/Robot.hpp>
#include <webots/supervisor.hpp>
#include <webots/keyboard.hpp>
// Added a new include file
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include "PID.h"
#include <windows.h>
#define TIME_STEP 8

/* Controller parameters */
#define PID_KP  34.0f
#define PID_KI  16.0f
#define PID_KD  5.0f

#define PID_TAU 0.02f

#define PID_LIM_MIN -20.0f
#define PID_LIM_MAX  20.0f

#define PID_LIM_MIN_INT -20.0f
#define PID_LIM_MAX_INT  20.0f

#define SAMPLE_TIME_S 1.0f

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
    Supervisor *robot = new Supervisor();
    Keyboard* keyboard = new Keyboard();
    keyboard->enable(TIME_STEP);
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
    PIDController pid = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
                          PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };

    PIDController_Init(&pid);
    float setpoint = 0.0f;
    double out = 0.0f;
 while (robot->step(TIME_STEP) != -1){
     double measurement = inertial->getRollPitchYaw()[0];
     switch (keyboard->getKey()) {
         case Keyboard::UP:
             setpoint = 0.1;
             break;
         case Keyboard::DOWN:
             setpoint = -0.1;
             break;
         default:
             setpoint = 0.0f;
     }
     PIDController_Update(&pid, setpoint, measurement);
     leftMotor->setVelocity(-pid.out);
     rightMotor->setVelocity(-pid.out);
     printf("setpoint:%f  roll:%f    Velocity:%f    out:%f \n", \
          setpoint, measurement,leftMotor->getVelocity(),-pid.out);
 };

 delete robot;

 return 0;
}