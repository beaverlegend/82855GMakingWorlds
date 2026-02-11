#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "device/mockimu.h"


#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

#define LEFT_FRONT_DRIVE -16
#define LEFT_MIDDLE_DRIVE 17
#define LEFT_BACK_DRIVE -18

#define RIGHT_FRONT_DRIVE 11 
#define RIGHT_MIDDLE_DRIVE -12
#define RIGHT_BACK_DRIVE 13

#define VERTICAL_ODOM -10
#define IMU 20


#define IntakeRight -1
#define IntakeLeft 10

#define TONGUE 'H' // change this
#define WINGS 'A'
#define INDEX 'E'
#define INTAKE 'G'

// #define DIST_FRONT 11
// #define DIST_BACK 12
// #define DIST_LEFT 13
// #define DIST_RIGHT 14

inline MockIMU imu(IMU, 359.0/ 360.0);
inline pros::MotorGroup left_mg({LEFT_FRONT_DRIVE, LEFT_MIDDLE_DRIVE, LEFT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_DRIVE, RIGHT_MIDDLE_DRIVE, RIGHT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup Intake_High_mg({IntakeRight, IntakeLeft});

// inline pros::MotorGroup Intake_Bottom({-IntakeTopRoller});
// inline pros::MotorGroup Intake_Top({-IntakeLastWheel});
inline pros::Rotation vertical_odom(VERTICAL_ODOM);

// pneumatics

inline pros::adi::Pneumatics tongue(TONGUE, false);
inline pros::adi::Pneumatics wings(WINGS, false);
inline pros::adi::Pneumatics indexer(INDEX, false);
inline pros::adi::Pneumatics intakeFinal(INTAKE, false);

// distance sensors
// inline pros::Distance dist_front(DIST_FRONT);
// inline pros::Distance dist_back(DIST_BACK);
// inline pros::Distance dist_left(DIST_LEFT);
// inline pros::Distance dist_right(DIST_RIGHT);