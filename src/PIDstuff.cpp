#include "PIDstuff.h"

// Constructor
PIDstuff::PIDstuff(int leftFrontMotorPort, int leftRearMotorPort, int rightFrontMotorPort, int rightRearMotorPort, 
                         int inertialPort, double wheelDiameter, double wheelRPM, 
                         double drivekP, double drivekI, double drivekD, double turnkP, double turnkI, double turnkD)
    : leftFrontMotor(leftFrontMotorPort, vex::gearSetting::ratio18_1, false),
      leftRearMotor(leftRearMotorPort, vex::gearSetting::ratio18_1, false),
      rightFrontMotor(rightFrontMotorPort, vex::gearSetting::ratio18_1, true),
      rightRearMotor(rightRearMotorPort, vex::gearSetting::ratio18_1, true),
      inertialSensor(inertialPort) {
    wheelCircumference = wheelDiameter * M_PI;
    this->wheelRPM = wheelRPM;

    // Set PID constants
    this->drivekP = drivekP;
    this->drivekI = drivekI;
    this->drivekD = drivekD;
    this->turnkP = turnkP;
    this->turnkI = turnkI;
    this->turnkD = turnkD;
}

// Drive a certain distance in inches using PID
void PIDstuff::driveDistance(double targetDistance) {
    double targetTicks = (targetDistance / wheelCircumference) * 360.0;
    double leftTicks = 0.0, rightTicks = 0.0;
    double leftIntegral = 0.0, rightIntegral = 0.0;
    double leftPreviousError = 0.0, rightPreviousError = 0.0;

    while (true) {
        double leftCurrent = (leftFrontMotor.position(vex::rotationUnits::deg) + leftRearMotor.position(vex::rotationUnits::deg)) / 2.0;
        double rightCurrent = (rightFrontMotor.position(vex::rotationUnits::deg) + rightRearMotor.position(vex::rotationUnits::deg)) / 2.0;

        double leftError = targetTicks - leftCurrent;
        double rightError = targetTicks - rightCurrent;

        double leftOutput = pidControl(targetTicks, leftCurrent, leftIntegral, leftPreviousError, drivekP, drivekI, drivekD);
        double rightOutput = pidControl(targetTicks, rightCurrent, rightIntegral, rightPreviousError, drivekP, drivekI, drivekD);

        leftFrontMotor.spin(vex::directionType::fwd, leftOutput, vex::velocityUnits::rpm);
        leftRearMotor.spin(vex::directionType::fwd, leftOutput, vex::velocityUnits::rpm);
        rightFrontMotor.spin(vex::directionType::fwd, rightOutput, vex::velocityUnits::rpm);
        rightRearMotor.spin(vex::directionType::fwd, rightOutput, vex::velocityUnits::rpm);

        if (fabs(leftError) < 5 && fabs(rightError) < 5) {
            break;
        }

        vex::task::sleep(20);
    }

    leftFrontMotor.stop();
    leftRearMotor.stop();
    rightFrontMotor.stop();
    rightRearMotor.stop();
}

// Turn to a specific angle using PID
void PIDstuff::turnToAngle(double targetAngle) {
    double currentAngle = inertialSensor.rotation();
    double integral = 0.0;
    double previousError = 0.0;

    while (true) {
        double error = targetAngle - currentAngle;
        double output = pidControl(targetAngle, currentAngle, integral, previousError, turnkP, turnkI, turnkD);

        leftFrontMotor.spin(vex::directionType::fwd, -output, vex::velocityUnits::rpm);
        leftRearMotor.spin(vex::directionType::fwd, -output, vex::velocityUnits::rpm);
        rightFrontMotor.spin(vex::directionType::fwd, output, vex::velocityUnits::rpm);
        rightRearMotor.spin(vex::directionType::fwd, output, vex::velocityUnits::rpm);

        if (fabs(error) < 2) {
            break;
        }

        currentAngle = inertialSensor.rotation();
        vex::task::sleep(20);
    }

    leftFrontMotor.stop();
    leftRearMotor.stop();
    rightFrontMotor.stop();
    rightRearMotor.stop();
}

// Split arcade drive function
void PIDstuff::splitArcadeDrive(double forward, double turn) {
    double left = forward + turn;
    double right = forward - turn;

    leftFrontMotor.spin(vex::directionType::fwd, left, vex::velocityUnits::pct);
    leftRearMotor.spin(vex::directionType::fwd, left, vex::velocityUnits::pct);
    rightFrontMotor.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
    rightRearMotor.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
}

void PIDstuff::toggle(vex::motor motor) {
    if (motor.isSpinning()) {
        motor.stop();
    } else {
        motor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
    
}
void PIDstuff::piston_toggle(vex::digital_out piston) {
    if (piston.value() == false) {
        piston.Set(true);
    } else {
        piston.Set(false);
    }
    
}
// PID control function
double PIDstuff::pidControl(double target, double current, double &integral, double previousError, double kP, double kI, double kD) {
    double error = target - current;
    integral += error;
    double derivative = error - previousError;
    previousError = error;

    return (kP * error) + (kI * integral) + (kD * derivative);
}