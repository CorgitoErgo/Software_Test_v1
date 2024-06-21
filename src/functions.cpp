#include "functions.h"
#include "definitions.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "main.h"
//#include "main.cpp"

pros::Imu imu_sensorL(imu_portL);
pros::Imu imu_sensorR(imu_portR);
pros::Motor lf(lf_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lb(lb_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rf(rf_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rb(rb_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Controller master(pros::E_CONTROLLER_MASTER);

void turn_pid(double target, bool turnLeft = false){
    double error, integral, derivative, prevError;
    double integral_threshold = 60;
    bool turnL, turnR = false;
    imu_sensorL.tare_heading();
    imu_sensorR.tare_heading();
    imu_reset = true;
    pros::delay(500);
    error = target - (headingValue + (imu_sensorL.get_heading() + imu_sensorR.get_heading())/2)/2;
    if (fabs(error) >= 180.0) {
        turnL = true;
    } else if (fabs(error) < 180.0) {
        turnR = true;
    }

    while(true){
        //error = target - imu_sensor.get_rotation();
        error = fabs(target - (headingValue + (imu_sensorL.get_heading() + imu_sensorR.get_heading())/2)/2);
        if (integral > integral_threshold) {
            integral = 0;
        } else{
            integral += error;
        }

        derivative = error - prevError;

        double power = fabs(turn_kP * error + turn_kI * integral + turn_kD * derivative);

        prevError = error;
        
        if (fabs(error) < 1.0) {
            lf.brake();
            lb.brake();
            rf.brake();
            rb.brake();
            lf.move_velocity(0);
            lb.move_velocity(0);
            rf.move_velocity(0);
            rb.move_velocity(0);
            break;
        }

        if(fabs(power)>=80)
			power = 80;

        if(turnR == true){
            lf.move_velocity(power*0.96);
            lb.move_velocity(power*0.96);
            rf.move_velocity(-power);
            rb.move_velocity(-power);
        }
        else{
            lf.move_velocity(-power*0.96);
            lb.move_velocity(-power*0.96);
            rf.move_velocity(power);
            rb.move_velocity(power);
        }

        pros::delay(20);
        //master.print(0,6,"%.2lf",headingValue);
    }
}

void turn_pid_int(double target, bool turnLeft = false){
    double error, integral, derivative, prevError;
    double integral_threshold = 60;
    bool turnL, turnR = false;
    //imu_sensorL.tare_heading();
    //imu_sensorR.tare_heading();
    pros::delay(10);
    error = target - (imu_sensorL.get_heading() + imu_sensorR.get_heading())/2;
    if (error >= 0) {
        turnR = true;
    } else if (error < 0) {
        turnL = true;
    }

    while(true){
        //error = target - imu_sensor.get_rotation();
        error = fabs(target - (imu_sensorL.get_heading() + imu_sensorR.get_heading())/2);
        if (integral > integral_threshold) {
            integral = 0;
        } else{
            integral += error;
        }

        derivative = error - prevError;

        double power = fabs(turn_kP * error + turn_kI * integral + turn_kD * derivative);

        prevError = error;
        
        if (fabs(error) < 1.0) {
            lf.brake();
            lb.brake();
            rf.brake();
            rb.brake();
            lf.move_velocity(0);
            lb.move_velocity(0);
            rf.move_velocity(0);
            rb.move_velocity(0);
            break;
        }

        if(fabs(power)>=80)
			power = 80;

        if(turnR == true){
            lf.move_velocity(power*0.96);
            lb.move_velocity(power*0.96);
            rf.move_velocity(-power);
            rb.move_velocity(-power);
        }
        else{
            lf.move_velocity(-power*0.96);
            lb.move_velocity(-power*0.96);
            rf.move_velocity(power);
            rb.move_velocity(power);
        }

        pros::delay(2);
        //master.print(0,6,"%.2lf",headingValue);
    }
}

void turn_pid_ext(double target, bool turnLeft = false){
    double error, integral, derivative, prevError;
    double integral_threshold = 60;
    bool turnL, turnR = false;
    //imu_sensor.tare();
    error = target - headingValue;
    if (fabs(error) >= 180.0) {
        turnL = true;
    } else if (fabs(error) < 180.0) {
        turnR = true;
    }
    //imu_reset = true;
    pros::delay(10);

    while(true){
        //error = target - imu_sensor.get_rotation();
        error = fabs(target - headingValue);
        if (integral > integral_threshold) {
            integral = 0;
        } else{
            integral += error;
        }

        derivative = error - prevError;

        double power = fabs(turn_kP * error + turn_kI * integral + turn_kD * derivative)/360 * 90;

        prevError = error;
        
        if (fabs(error) < 1.0) {
            lf.brake();
            lb.brake();
            rf.brake();
            rb.brake();
            lf.move_velocity(0);
            lb.move_velocity(0);
            rf.move_velocity(0);
            rb.move_velocity(0);
            break;
        }

        if(fabs(power)>=80)
			power = 80;

        if(turnR == true){
            lf.move_velocity(power*0.96);
            lb.move_velocity(power*0.96);
            rf.move_velocity(-power);
            rb.move_velocity(-power);
        }
        else if(turnL == true){
            lf.move_velocity(-power*0.96);
            lb.move_velocity(-power*0.96);
            rf.move_velocity(power);
            rb.move_velocity(power);
        }

        pros::delay(10);
        //master.print(0,6,"%.2lf",headingValue);
    }
}

