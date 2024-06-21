#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "functions.h"
#include "pros/serial.h"
#include "pros/serial.hpp"
#include <sstream>	        //Include sstream for serial parsing
#include <vector>
#include "spline-master/src/spline.h"

//Prototypes for hidden vex functions to bypass PROS bug
extern "C" int32_t vexGenericSerialReceive( uint32_t index, uint8_t *buffer, int32_t length );
extern "C" void vexGenericSerialEnable(  uint32_t index, uint32_t nu );
extern "C" void vexGenericSerialBaudrate(  uint32_t index, uint32_t rate );
extern "C" int32_t vexGenericSerialTransmit( uint32_t index, uint8_t *buffer, int32_t length );

//C2 Splines
std::vector<double> X = {0.1, 0.4, 1.2, 1.8, 2.0}; // must be increasing
std::vector<double> Y = {0.1, 0.7, 0.6, 1.1, 0.9};

tk::spline s(X,Y);			// X needs to be strictly increasing
double value=s(1.3);		// interpolated value at 1.3
double deriv=s.deriv(1,1.3);	// 1st order derivative at 1.3

//UART COMMS
#define SERIALPORT 19

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
    pros::Imu imu_sensorL(imu_portL);
    pros::Imu imu_sensorR(imu_portR);
    
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
    imu_sensorL.tare_heading();
    imu_sensorR.tare_heading();
    imu_reset = true;
    while (true) {
        
        // Buffer to store serial data
        uint8_t buffer[256];
        int len = 256;
        
        // Get serial data
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, len);
        //master.print(0,0,"%d",nRead);
        // Now parse the data
        if(master.get_digital(DIGITAL_Y) || imu_reset == true){
            vexGenericSerialTransmit(SERIALPORT - 1, (uint8_t *)msg3, 2);
            distX = 0;
            distY = 0;
            headingValue = 0;
            readSerial = true;
            imu_reset = false;
            imu_sensorL.tare_heading();
            imu_sensorR.tare_heading();
            pros::delay(70);
        }
        
        if (nRead >= 0 && readSerial) {
            
            // Stream to put the characters in
            std::stringstream myStream("");
            std::stringstream myStream2("");
            std::stringstream myStream3("");
            bool recordAngle = false;
            bool recordOpticalX = false;
            bool recordOpticalY = false;
            
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
                    pros::screen::print(TEXT_MEDIUM, 1, "[External IMU Heading]");
                    pros::screen::print(TEXT_MEDIUM, 2, "Degrees: %.2lf", headingValue);
                    pros::screen::print(TEXT_MEDIUM, 3, "Left VEX IMU: %.2lf", imu_sensorL.get_heading());
                    pros::screen::print(TEXT_MEDIUM, 4, "Right VEX IMU: %.2lf", imu_sensorR.get_heading());
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
        pros::delay(10);
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
    lf.set_brake_mode(MOTOR_BRAKE_HOLD);
    lb.set_brake_mode(MOTOR_BRAKE_HOLD);
    rf.set_brake_mode(MOTOR_BRAKE_HOLD);
    rb.set_brake_mode(MOTOR_BRAKE_HOLD);
    
	pros::Imu imu_sensorL(imu_portL);
    pros::Imu imu_sensorR(imu_portR);
    imu_sensorL.set_data_rate(5);
    imu_sensorR.set_data_rate(5);
    pros::Serial serial(19);
    int32_t serial_enable(19);
    serial.set_baudrate(115200);
  	while(!imu_sensorL.reset(true));
    while(!imu_sensorR.reset(true));
    pros::Task gyroTask (serialRead);
	pros::delay(15);
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
    double currDistY = fabs(distY);

    while(!lf.tare_position());
    while(!rf.tare_position());

    while(true){
        //error = target - ((((fabs(lf.get_position()) + fabs(rf.get_position())) / 2) * distance_per_deg));
        //printf("%.1lf\n", ((((fabs(lf.get_position()) + fabs(rf.get_position())) / 2) * distance_per_deg)));
        error = target - (fabs(distY - currDistY));

        if (integral > integral_threshold) {
            integral += error;
        } else{
            integral = 0;
        }

        derivative = error - prevError;

        double power = drive_kP * error + drive_kI * integral + drive_kD * derivative;

        prevError = error;

        if (fabs(error) < 5) {
            lf.move(-5);
            lb.move(-5);
            rf.move(-5);
            rb.move(-5);
            lf.brake();
            lb.brake();
            rf.brake();
            rb.brake();
            lf.move(0);
            lb.move(0);
            rf.move(0);
            rb.move(0);
            break;
        }
        else if (error < 0){
            lf.move(-5);
            lb.move(-5);
            rf.move(-5);
            rb.move(-5);
            lf.brake();
            lb.brake();
            rf.brake();
            rb.brake();
            lf.move(0);
            lb.move(0);
            rf.move(0);
            rb.move(0);
            break;
        }

		if(fabs(power)>=40)
			power = 40;

        if(forward){
            lf.move(power*0.36);
            rf.move(power*0.36);
            rb.move(power*1.29);
            lb.move(power*1.29);
        }
        else{
            lf.move(-power*0.36);
            rf.move(-power*0.36);
            rb.move(-power*1.29);
            lb.move(-power*1.29);
        }

        pros::delay(2);
    }
}

void autonomous() {
	//turn_pid(90.0, false);
    //pros::delay(1000);
    //turn_pid_int(90.0, false);
    //pros::delay(1000);
    //turn_pid_ext(90.0, false);
    //pros::delay(1000);
    //drive_pid(100, true);
    //pros::delay(100);
	//drive_pid(100);
    /*squiggles::Constraints constraints = squiggles::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);
    squiggles::SplineGenerator generator = squiggles::SplineGenerator(
    constraints,
    std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraints));

    std::vector<squiggles::ProfilePoint> path = generator.generate({
    squiggles::Pose(0.0, 0.0, 1.0),
    squiggles::Pose(4.0, 4.0, 1.0)});

    auto chassis = ChassisControllerBuilder()
        .withMotors({lf_motor, lb_motor}, {rf_motor, rb_motor}) // Ports for the left and right motors
        .withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 6.3_in}, imev5BlueTPR})
        .build();

    std::shared_ptr<ChassisController> chassisController = ChassisControllerBuilder()
        .withMotors({1, 2}, {-3, -4})
        .withDimensions(AbstractMotor::gearset::green, {{4_in, 12_in}, imev5GreenTPR})
        .build();

    auto profileController = AsyncMotionProfileControllerBuilder()
        .withLimits({
            1.0, // Maximum linear velocity of the Chassis in m/s
            2.0, // Maximum linear acceleration of the Chassis in m/s^2
            10.0 // Maximum linear jerk of the Chassis in m/s^3
        })
        .withOutput(chassis)
        .buildMotionProfileController();

    profileController->generatePath({{0_ft, 0_ft, 0_deg}, {2_ft, 2_ft, 90_deg}}, "A");

    profileController->setTarget("A");
    profileController->waitUntilSettled();*/
    drive_pid(6000, true);
}

void autonomous1() {
	//turn_pid(90.0, false);
    //pros::delay(1000);
    //turn_pid_int(90.0, false);
    //pros::delay(1000);
    //turn_pid_int(90.0, false);
    //pros::delay(1000);
	//drive_pid(100);
}

void autonomous2() {
	//turn_pid(90.0, false);
    //pros::delay(1000);
    //turn_pid_int(90.0, false);
    //pros::delay(1000);
    //turn_pid_int(90.0, false);
    //pros::delay(1000);
    //drive_pid(100, true);
    //pros::delay(100);
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
        if(master.get_digital(DIGITAL_A))
			autonomous1();
        if(master.get_digital(DIGITAL_B))
            autonomous2();
		
		left = power + turn;
		right = power - turn;
		lf.move(left*0.36);
		lb.move(left*0.36);
		rf.move(right*1.29);
		rb.move(right*1.29);

		pros::delay(2);
	}
}