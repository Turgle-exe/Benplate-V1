#include "vex.h"
#include "PIDstuff.h"

vex::competition Competition;
//Welcome to BENPLATE the best template ever
//Created by: Ben DeCosta 119B on 3/2/2025
vex::controller Controller1 = vex::controller();
// Define the robot with motor ports, sensor ports, wheel diameter, wheel RPM, and PID constants
PIDstuff robot(

//these are the motor ports for your drive train
//change these to the ports that your drive motors are plugged into
//LEFT MOTORS
1, 2, 
//RIGHT MOTORS
3, 4, 

//change this to the port that your inertial sensor is plugged into
5, 

//This is the diameter of your wheels in inches(if its a 4 inch wheel put 4.125)
4.125, 

//this is the RPM of your drive
200.0, 
//YOUR SETUP IS COMPLETE


//these are the PID constants for Drive PID you dont need to mess with these
0.5, 0.0, 0.1,


//IF YOUR TURNING IS WOOBBLY CHANGE THESE VALUES


//this is your P constant START HERE

//if your robot looks like it is turning too far and wobbling lower this value untill it only overshoots by a little bit
0.5, 
//next go to the D constant


//this is your I constant

//you shouldnt need to change this value if you really need to look up how to tune I constants or contact me(bendecosta09@gmail.com)
0.0,


//this is your D constant

//after tuning your P constant tune this value until it stops overshooting completly lastly go to the I constant
0.1);


//USE THIS AREA TO DEFINE ANY MOTORS/SENSORS:
vex::motor intake = vex::motor(vex::PORT8);
vex::digital_out piston = vex::digital_out("A");
vex::inertial inertialSensor = vex::inertial(vex::PORT5);




//....
void pre_auton() {
    //this is for your pre-auton do not change this
    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating()) {
        vex::task::sleep(20);
    }

}

void autonomous() {
    robot.turnToAngle(90.0);
    robot.turnToAngle(-90.0);
    robot.turnToAngle(0.0);
}

void usercontrol() {
    while (true) {
        //this is for your driving do not change this
        double forward = Controller1.Axis3.position();
        double turn = Controller1.Axis1.position();
        robot.splitArcadeDrive(forward, turn);
        //PUT YOUR DRIVER CONTROL CODE HERE:

        //example code for toggling a intake
        if (Controller1.ButtonR1.pressing()) {
            robot.toggle(intake);
        }

        //example code for toggling a psiton
        if (Controller1.ButtonR1.pressing()) {
            robot.piston_toggle(psiton);
        }




        vex::task::sleep(20);
    }
}

int main() {
    //you don't need to change this
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);



    while (true) {
        vex::task::sleep(100);
    }
}