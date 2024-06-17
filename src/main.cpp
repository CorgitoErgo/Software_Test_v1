#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "functions.h"
#include "pros/serial.h"
#include "pros/serial.hpp"
#include <sstream>	// Include sstream for serial parsing

// Prototypes for hidden vex functions to bypass PROS bug
extern "C" int32_t vexGenericSerialReceive( uint32_t index, uint8_t *buffer, int32_t length );
extern "C" void vexGenericSerialEnable(  uint32_t index, uint32_t nu );
extern "C" void vexGenericSerialBaudrate(  uint32_t index, uint32_t rate );
extern "C" int32_t vexGenericSerialTransmit( uint32_t index, uint8_t *buffer, int32_t length );

// Port to use for serial data
#define SERIALPORT 19
// Variable to put the heading value into
double headingValue = 0;
double distX = 0;
double distY = 0;
char msg[1] = {'R'};
char msg2[1] = {'T'};
char msg3[2] = {'W', 'B'};
bool readSerial = true;
bool imu_reset = false;

void serialRead(void* params) {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Imu imu_sensor(imu_port);
    
    // Start serial on desired port
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    
    // Set BAUD rate
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    
    // Let VEX OS configure port
    pros::delay(10);
    pros::screen::set_pen(COLOR_BLUE);
    
    // Serial message format:
    // D[LIDAR DIST]I[IR DATA]A[GYRO ANGLE]E
    // Example Message:
    // D50.2I128A12.32E
    bool toggle = false;
    //pros::screen::print(TEXT_MEDIUM, 6, "TEST");
    imu_sensor.tare();
    imu_reset = true;
    while (true) {
        
        // Buffer to store serial data
        uint8_t buffer[256];
        int len = 256;
        
        // Get serial data
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, len);
        //master.print(0,0,"%d",nRead);
        // Now parse the data
        
        if (nRead >= 0 && readSerial) {
            
            // Stream to put the characters in
            std::stringstream myStream("");
            std::stringstream myStream2("");
            std::stringstream myStream3("");
            bool recordAngle = false;
            bool recordOpticalX = false;
            bool recordOpticalY = false;
            if(master.get_digital(DIGITAL_Y) || imu_reset == true){
                vexGenericSerialTransmit(SERIALPORT - 1, (uint8_t *)msg3, 2);
                distX = 0;
                distY = 0;
                headingValue = 0;
                readSerial = true;
                imu_reset = false;
                imu_sensor.tare_heading();
                pros::delay(20);
            }
            
            // Go through characters
            for (int i = 0; i < nRead; i++) {
                // Get current char
                char thisDigit = (char)buffer[i];
                
                // If its special, then don't record the value
                if (thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X' || thisDigit == 'C' || thisDigit == 'Y' || thisDigit == 'D'){
                    recordAngle = false;
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                
                // Finished recieving angle, so put into variable
                if (thisDigit == 'E') {
                    recordAngle = false;
                    myStream >> headingValue;
                    pros::screen::print(TEXT_MEDIUM, 1, "[IMU Heading]");
                    pros::screen::print(TEXT_MEDIUM, 2, "degrees: %.2lf", headingValue);
                    pros::screen::print(TEXT_MEDIUM, 3, "int degrees: %.2lf", imu_sensor.get_heading());
                    //headingValue = 0;
                    //master.print(0,0,"%f",headingValue);
                    //pros::screen::print(TEXT_MEDIUM, 3, "%f", headingValue);
                }

                if (thisDigit == 'C'){
                    recordOpticalX = false;
                    myStream2 >> distX;
                    //pros::screen::print(TEXT_LARGE, 3, "[Optical Flow]");
                    pros::screen::print(TEXT_MEDIUM, 6, "distX: %.2lf", distX);
                }

                if (thisDigit == 'D'){
                    recordOpticalY = false;
                    myStream3 >> distY;
                    pros::screen::print(TEXT_MEDIUM, 5, "[Optical Flow]");
                    pros::screen::print(TEXT_MEDIUM, 7, "distY: %.2lf", (-distY));
                }
                
                // If we want the digits, put them into stream
                if (recordAngle)
                    myStream << (char)buffer[i];
                
                if (recordOpticalX)
                    myStream2 << (char)buffer[i];

                if (recordOpticalY)
                    myStream3 << (char)buffer[i];
                
                // If the digit is 'A', then the following data is the angle
                if (thisDigit == 'A'){
                    recordAngle = true;
                }
                
                if (thisDigit == 'X')
                    recordOpticalX = true;
                
                if (thisDigit == 'Y')
                    recordOpticalY = true;
                
                //myStream >> headingValue;
                //master.print(0,0,"%f",headingValue);
                    
                
            }
            
        }
    
        // Delay to let serial data arrive
        pros::delay(20);
        //master.print(0, 6, "%.2lf", headingValue);
        pros::Task::delay(15);
        /*if(toggle){
            vexGenericSerialTransmit(SERIALPORT - 1, (uint8_t *)msg, 1);
            toggle = !toggle;
        }
        else if(!toggle){
            vexGenericSerialTransmit(SERIALPORT - 1, (uint8_t *)msg2, 1);
            toggle = !toggle;
        }     */
    }
    
}

void initialize() {
	pros::Motor lf(lf_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lb(lb_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rf(rf_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rb(rb_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Imu imu_sensor(imu_port);
    pros::Serial serial(19);
    int32_t serial_enable(19);
    serial.set_baudrate(115200);
  	imu_sensor.reset(true);
    pros::Task gyroTask (serialRead);
	pros::delay(5);
}

void disabled() {}

void competition_initialize() {}

void drive_pid(int target, bool forward = true){
	pros::Motor lf(lf_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lb(lb_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rf(rf_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rb(rb_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    double error, integral, derivative, prevError;
    float integral_threshold = 100;

    lf.tare_position();
    rf.tare_position();

    while(true){
        error = target - (((fabs(lf.get_encoder_units()) + fabs(rf.get_encoder_units())) / 2) * distance_per_deg);

        if (integral > integral_threshold) {
            integral += error;
        } else{
            integral = 0;
        }

        derivative = error - prevError;

        double power = drive_kP * error + drive_kI * integral + drive_kD * derivative;

        prevError = error;

        if (fabs(error) < 5) {
            break;
        }

		if(fabs(power)>300)
			power = 300;

        if(forward){
            lf.move_velocity(power);
            rf.move_velocity(power);
            rb.move_velocity(power);
            lb.move_velocity(power);
        }
        else{
            lf.move_velocity(-power);
            rf.move_velocity(-power);
            rb.move_velocity(-power);
            lb.move_velocity(-power);
        }

        pros::delay(2);
    }
}

void autonomous() {
	//turn_pid(90.0, false);
    //pros::delay(1000);
    //turn_pid_int(90.0, false);
    //pros::delay(1000);
    turn_pid_ext(90.0, false);
    pros::delay(1000);
	//drive_pid(100);
}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor lf(lf_motor);
	pros::Motor lb(lb_motor);
	pros::Motor rf(rf_motor);
	pros::Motor rb(rb_motor);
    pros::Serial serial(19);
    serial.flush();
	
	double power, turn;
	double left, right;
	//master.clear();
	//pros::Imu imu_sensor(imu_port);
    //pros::Task gyroTask (serialRead);
    //headingValue = 1;
	while (true) {
		power = master.get_analog(ANALOG_LEFT_Y);
		turn = master.get_analog(ANALOG_RIGHT_X);
		if(master.get_digital(DIGITAL_X))
			autonomous();
        
		
		left = power + turn;
		right = power - turn;
		lf.move(left);
		lb.move(left);
		rf.move(right);
		rb.move(right);

		pros::delay(2);
	}
}