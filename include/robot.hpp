#pragma once
#include "api.h"
#include "lemlib/api.hpp"

#ifdef __cplusplus
// controller
extern pros::Controller controller;

// motor groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor Intake;
extern pros::Motor Outtake;
extern pros::adi::Pneumatics Loader;
extern pros::adi::Pneumatics goal_switch;
extern pros::adi::Pneumatics descore;

extern pros::Distance Right_Distance_sensor;
extern pros::Distance Left_Distance_sensor;
extern pros::Distance Front_Distance_sensor;
extern pros::Distance Back_Distance_sensor;

// Inertial Sensor
extern pros::Imu imu_sensor;

// tracking wheels
extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;
extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;

// drivetrain settings
extern lemlib::Drivetrain drivetrain;

// lateral motion controller
extern lemlib::ControllerSettings linearController;

// angular motion controller
extern lemlib::ControllerSettings angularController;

// sensors for odometry
extern lemlib::OdomSensors sensors;

// input curves
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;

// create the chassis
extern lemlib::Chassis chassis;

// individual motors
extern pros::Motor LF;
extern pros::Motor LB;
extern pros::Motor LU;
extern pros::Motor RF;
extern pros::Motor RB;
extern pros::Motor RU;
#endif
