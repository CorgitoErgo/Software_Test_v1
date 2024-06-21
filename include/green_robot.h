#ifndef _definitions_h_
#define _definitions_h_

#include "geometry/pose.hpp"
#include "squiggles.hpp"
#include "okapi/api.hpp"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"
#include "okapi/api/control/util/pathfinderUtil.hpp"
using namespace okapi;

#define PI 3.1415926
#define wheel_travel (PI/180) * 360     //radians

#define wheel_diameter 69.85            //in mm
#define wheel_radius 34.925
#define distance_per_rev 219.440410     //circumference
#define distance_per_deg 0.609556694 

#define lf_motor 9
#define lb_motor 18
#define rf_motor 10
#define rb_motor 20
#define imu_portL 7
#define imu_portR 6

//turning PID
#define turn_kP 1.1
#define turn_kI 0
#define turn_kD 0

//drive PID
#define drive_kP 1
#define drive_kI 0
#define drive_kD 0

//Splines
#define RPS              600 / 60       // rev per second (600rpm / 60sec)
#define Velocity         (distance_per_rev/100) * RPS     // m/s
#define Force            0.3 / ((wheel_diameter/100)/2)   // per wheel
#define Total_force      Force * 4
#define Acceleration     Total_force / 2.5 //2kg robot

const double MAX_VEL     = 2.0; // in meters per second
const double MAX_ACCEL   = 3.0; // in meters per second per second
const double MAX_JERK    = 6.0; // in meters per second per second per second
const double ROBOT_WIDTH = 0.4; // in meters

#endif