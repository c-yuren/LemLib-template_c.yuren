# Robot Custom Functions & LemLib Manual

[🇹🇼 中文版本 (Chinese)](README.md) | [🇺🇸 English Version](README_en.md)

This manual documents the chassis control configuration, PID tuning guide, and the usage of all custom functions in this project for writing autonomous programs (Auton).

## Table of Contents
1. [LemLib Basic Setup (main.cpp)](#1-lemlib-basic-setup-maincpp)
2. [LemLib PID Tuning Guide](#2-lemlib-pid-tuning-guide)
3. [LemLib Movement Commands](#3-lemlib-movement-commands)
4. [Custom Functions (functions.cpp)](#4-custom-functions-functionscpp)
    - Basic Debugging
    - Mechanism Control
    - Time-based Movement
    - Sensor-based Movement
    - Relative Coordinate Movement
    - Positioning and Wall Alignment

---

## 1. LemLib Basic Setup (main.cpp)

To make the robot function properly, we must first configure the hardware and controller parameters in `main.cpp`.

### 1-1. Defining Motor Groups
Group the motors driving the chassis into left and right groups. A negative sign means the motor is reversed, and a positive sign means forward. This must correspond to the physical gear transmission direction.
```cpp
pros::MotorGroup leftMotors({-1, -2, 3}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({4, 5, -6}, pros::MotorGearset::blue);
```

### 1-2. Drivetrain Parameters
Set the track width and RPM so the system can convert coordinates into actual motor rotations.
```cpp
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 
                              11,           // Track width (inches)
                              lemlib::Omniwheel::NEW_325, // Wheel type and diameter
                              450,          // Calculated actual RPM after gear ratio
                              2             // Chase power/drift friction coefficient
);
```

### 1-3. Creating the Chassis Object
Bind the sensors (gyroscope and tracking wheels) and create the `chassis` object.
```cpp
lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal, nullptr, &imu_sensor);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
```

---

## 2. LemLib PID Tuning Guide

In `main.cpp`, there are two `ControllerSettings` (linear and angular). PID parameters determine the accuracy and smoothness of the robot's movement.

### Parameter Explanation
- **kP (Proportional):** Pushing power. If too small, it won't reach the target; if too large, it oscillates violently back and forth.
- **kI (Integral):** Usually set to 0. Rarely used for VEX chassis.
- **kD (Derivative):** Braking damper. Used to counteract the oscillation caused by kP.
- **Slew:** Starting acceleration limit. A smaller value means a softer start, 0 implies no limit.

### Tuning Steps
1. Set kI and kD to 0.
2. Slowly increase kP and run the robot for a short distance or a 90-degree turn. Keep increasing kP until the robot starts to oscillate back and forth noticeably upon reaching the target.
3. Keep that kP value and start increasing the kD value until the oscillation disappears and the robot stops stably and accurately.
4. Common reference values:
   - Linear: kP is usually around 10, kD is several times kP, and Slew can be set to 20 to prevent wheels from slipping on startup.
   - Angular: kP is usually between 2 and 5, and kD is usually larger (e.g., between 7 and 35).

---

## 3. LemLib Movement Commands

### Setting Initial Position
The required first line of any Auton program. Tells the system where the robot is initially placed on the field's absolute coordinates.
```cpp
chassis.setPose(0, 0, 90); // Coordinate (0,0), facing right 90 degrees
```

### Absolute Coordinate Movement
The system will automatically turn, accelerate, and brake to reach the specified absolute point.
```cpp
chassis.moveToPoint(0, 24, 2000); // Move to (0, 24) in front, timeout 2000ms

// To move backwards to the point, append params and set forwards to false
chassis.moveToPoint(0, -24, 2000, {.forwards = false});
```

### Pivot Turn
Specify the robot to face a specific absolute angle or aim directly at a specific coordinate.
```cpp
chassis.turnToHeading(180, 1000); // Turn to face the back
chassis.turnToPoint(24, 24, 1000); // Turn to aim at (24,24)

// Aim at an object with the back of the robot
chassis.turnToPoint(24, 24, 1000, {.forwards = false}); 
```

---

## 4. Custom Functions (functions.cpp)

Helper functions packaged internally in this project.

### Basic Debugging
- `print_coord()`: Prints the current X, Y coordinates to the Terminal for debugging positioning issues.

### Mechanism Control
- `Outtaking(int timeMsec, bool blocking)`
  Controls Intake/Outtake to spin for a specified `timeMsec`.
  Includes an anti-jam mechanism that automatically reverses slightly to clear jams.
  Set `blocking` to `true` to finish the action in place before running the next line; set to `false` for background execution (can intake while moving).

### Time-Based Movement
- `moveTime(double speed_volt, double Timemsec, bool stop)`
  Ignores sensors and gives voltage to the motor purely based on time.
  Suitable for rough short-distance movements or pushing against a wall.

### Sensor-Based Movement
- `drive_distance_front(float distance, float heading, float max_voltage, float timeout)`
- `drive_distance_back(float distance, float heading, float max_voltage, float timeout)`
  Moves straight until the front (or back) laser distance sensor reading is less than `distance`.
  Forcibly locks the `heading` angle to prevent drifting during the movement.

### Relative Coordinate Movement
- `drive_distance(float distance, float heading, int timeout, lemlib::MoveToPointParams params, bool async)`
  No need to calculate the endpoint coordinate separately. Moves `distance` distance towards the `heading` angle relative to the current position (internally uses moveToPoint).
- `driveToWallSmart(float targetGapInches, int timeout, bool useFrontSensor, ...)`
  Combines distance sensors with LemLib coordinates. Reads the distance to the wall and offsets the absolute coordinates to drive smoothly to a position `targetGapInches` inches away from the calculated wall.

### Positioning and Wall Alignment
- `reset(SensorSelect s1, SensorSelect s2, double angle)`
  Uses laser distance to adjacent walls to recalculate and correct the robot's system coordinates (X or Y).
  You can choose only one wall (e.g., `front`), filling the other with `none`.
  Put the currently known true angle in `angle`. If `-1000`, the current IMU data is used directly.
  ```cpp
  // Example: Wall alignment, read the front wall to correct positioning
  reset(front, none, -1000);
  ```
