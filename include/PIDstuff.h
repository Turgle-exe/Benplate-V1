#ifndef VEXPIDDRIVE_H
#define VEXPIDDRIVE_H

#include "vex.h"

class PIDstuff {
public:
    // Constructor
    PIDstuff(int leftFrontMotorPort, int leftRearMotorPort, int rightFrontMotorPort, int rightRearMotorPort, 
                int inertialPort, double wheelDiameter, double wheelRPM, 
                double drivekP, double drivekI, double drivekD, double turnkP, double turnkI, double turnkD);

    // Drive a certain distance in inches using PID
    void driveDistance(double targetDistance);

    // Turn to a specific angle using PID
    void turnToAngle(double targetAngle);

    // Split arcade drive function
    void splitArcadeDrive(double forward, double turn);

    void toggle(vex::motor motor);

private:
    // Motor and sensor objects
    vex::motor leftFrontMotor;
    vex::motor leftRearMotor;
    vex::motor rightFrontMotor;
    vex::motor rightRearMotor;
    vex::inertial inertialSensor;

    // Wheel properties
    double wheelCircumference;
    double wheelRPM;

    // PID constants for driving and turning
    double drivekP, drivekI, drivekD;
    double turnkP, turnkI, turnkD;

    // PID control function
    double pidControl(double target, double current, double &integral, double previousError, double kP, double kI, double kD);
};

#endif // VEXPIDDRIVE_H