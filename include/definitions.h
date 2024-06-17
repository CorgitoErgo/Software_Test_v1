#ifndef _definitions_h_
#define _definitions_h_

#define PI 3.1415926
#define wheel_travel (PI/180) * 360 //radians
//in mm
#define wheel_diameter 69.85
#define wheel_radius 34.925
#define distance_per_rev 219.440410
#define distance_per_deg 0.609556694

#define lf_motor 20
#define lb_motor 10
#define rf_motor 11
#define rb_motor 1
#define imu_port 7

//turning PID
#define turn_kP 1.87
#define turn_kI 0.1
#define turn_kD 0.15

//drive PID
#define drive_kP 6
#define drive_kI 0.01
#define drive_kD 0.05

#endif