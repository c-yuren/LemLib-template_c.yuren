#include "main.h"
#include "auton.h"

void skill_auton(){
        
        Intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Outtake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        descore.set_value(1);
        chassis.setPose(-50.000000, 18,90);
        chassis.moveToPoint(-23, 25, 2000);
        Intake.move(65);
        pros::delay(50);
        chassis.turnToHeading(315.0, 1150);
        chassis.moveToPoint(-12, 12, 2305, {.forwards = false});
    moveTime(-127,600);
        pros::delay(50);
        Intake.move(127);
    moveTime(-127,700);
        Outtake.move(65);
        pros::delay(1000);
        Outtake.move(0);

        Loader.set_value(1);
        chassis.moveToPoint(-48.0, 48.2, 2323, {.maxSpeed = 100});
        pros::delay(50);
        chassis.turnToHeading(270.0, 1500, {.maxSpeed=90});
            // /*loader_1*/ 
        moveTime(110,1000,false);
        moveTime(30,800);
        moveTime(-20,300);
        moveTime(30,800);
        moveTime(-20,300);
        moveTime(30,800);
            drive_distance_front(42, -90, 50, 1000);
            Loader.set_value(0);
            pros::delay(50);
            chassis.turnToHeading(25, 1150);
            chassis.moveToPoint(-34, 65, 1000, {.forwards = 1, .maxSpeed = 70});
            chassis.turnToHeading(90, 1100);
            chassis.moveToPoint(30, 66, 3000);
            pros::delay(50);
            chassis.turnToHeading(0,1300);
            chassis.waitUntilDone();
            drive_distance_front(43,359,30,1500);

            chassis.turnToHeading(90,1300);
            chassis.waitUntilDone();
            
// long goal+Loader 2 
   moveTime(-80,1000);
   Outtaking(3500,0);
    moveTime(-35,3000);
    // chassis.setPose( 34, 48, 90);
    chassis.setPose( 34, 48, 90);
    Loader.set_value(1);
    drive_distance(30,90,2000,{.maxSpeed=100});
    drive_distance(-2,90,3000);
    drive_distance(4,90,700); 
    pros::delay(700);
    drive_distance(-30,90,1500);
    Loader.set_value(0);
    
    Outtaking(3000,0);
    moveTime(-20,3000);
    Outtake.move(0);
    chassis.turnToHeading(115,1300);
    drive_distance(34,115,2500);
    // chassis.moveToPoint(69,32,2500);
    chassis.swingToHeading (175,DriveSide::RIGHT,1500);
    moveTime(60,3000);
    moveTime(127,850,false);
    if(Front_Distance_sensor.get_distance()<=1300) chassis.tank(0,0);
    drive_distance_front(110,180,70);
printf("%.1f",Front_Distance_sensor.get_distance());
    chassis.turnToHeading(90,1600);

    drive_distance_front(120,90,75,3000);
printf("%.1f",Front_Distance_sensor.get_distance());
    drive_distance_front(115,90,20,500);
printf("%.1f",Front_Distance_sensor.get_distance());
    chassis.setPose(24,-24,90);
    pros::delay(100);
    chassis.turnToHeading(135,1300);
    // drive_distance(-10,135,2000);

    pros::delay(100);
    // chassis.moveToPoint(20,-20,2000,{.forwards=0});
    drive_distance(-18,135,2000);
    Intake.move(100);
    Outtake.move(100);
    pros::delay(500);
    Outtake.move(0);
    pros::delay(2500);
    Intake.move(120);
/*loader 3*/ chassis.moveToPoint(48.0, -48.2, 2323, {.maxSpeed = 100});
    

    
    chassis.turnToHeading(90,1000);
    Intake.move(100);
    
    Loader.set_value(1);
    drive_distance(20,90,1000);
    drive_distance(-2,90,3000);
    drive_distance(4,90,700); 
    pros::delay(700);
    
            drive_distance_front(42, 90, 50, 1000);
            Loader.set_value(0);
            pros::delay(500);
            chassis.turnToHeading(205, 1500);
            chassis.moveToPoint(34, -65, 1000, {.forwards = 1, .maxSpeed = 70});
            chassis.turnToHeading(-90, 1200);
            chassis.moveToPoint(-30, -66, 3000);
            pros::delay(50);
            chassis.turnToHeading(180, 1200);
        drive_distance_front(44,180,40,2000);
        printf("%.1f",Front_Distance_sensor.get_distance());
        chassis.turnToHeading(270, 1200);
        chassis.waitUntilDone();
            
//long goal 2
    moveTime(-127,1500);
    Outtaking(3000,0);
    moveTime(-35,3000);
    Intake.move(100);
    chassis.setPose( -34, -48, chassis.getPose().theta);
    Loader.set_value(1);
    drive_distance(18,90,2200,{.earlyExitRange=1});

    moveTime(65,700);
    moveTime(30,700);
    moveTime(-20,300);
    moveTime(30,800);
    moveTime(-20,300);
    moveTime(30,800);
    chassis.moveToPoint(-35, -48, 2293, {.forwards = 0});
    Loader.set_value(0);
    
    moveTime(-50,700);
    Outtake.move(127);
    moveTime(-20,3000);
    Outtake.move(0);
    chassis.turnToHeading(-25, 1500);
    chassis.moveToPoint(34, -65, 1000, {.forwards = 1, .maxSpeed = 70});
    chassis.turnToHeading(90, 1200);
    chassis.moveToPoint(-24, -60, 3000);
    drive_distance_front(40, 90, 30, 2000);
    pros::delay(50);
    chassis.turnToHeading(0.0, 1300);
    drive_distance_front(45, 0, 25);
    chassis.turnToHeading(90.0, 1500);
   Intake.move(127);
    chassis.turnToHeading(115,600);
    chassis.moveToPoint(63,38,2500);
    chassis.swingToHeading(174,DriveSide::RIGHT,1500);
    moveTime(20,200);
    moveTime(60,800);
}

void skill_driver(){
    descore.set_value(1);
    chassis.setPose(-50.000000, 18, 88.183000);

    Intake.move(65);
    chassis.moveToPoint(-24, 24, 2000);
    // /*path*/chassis.follow(skill_0114_path_1_txt, 8.67166, 2360);
    pros::delay(50);
    chassis.turnToHeading(315.0, 1300);
    chassis.moveToPoint(-12, 12, 2305, {.forwards = false});
moveTime(-127,600);
    pros::delay(50);
    Intake.move(127);
moveTime(-127,700);
    Outtake.move(65);
    pros::delay(1000);
    Outtake.move(0);
    Loader.set_value(1);
}