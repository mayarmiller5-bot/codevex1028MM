#include "main.h"
//#include "api.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "pros/adi.hpp"
//#include <utility>
bool piston_park_val = false;
bool piston_val = false;
bool piston_score_val = false;
pros::Motor RightFront (6);
pros::Motor RightMiddle (7);
pros::Motor RightBack (17);
pros::Motor LeftBack (11);
pros::Motor LeftMiddle (18);
pros::Motor LeftFront (4);



pros::Motor intake1 (12); //1st inatake
pros::Motor intake2 (3); // reverse inatake
pros::Motor intake_score(8);



pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({4, 18, 11});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({6, 7, 17});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


// create an imu on port 10
pros::Imu imu(10);
pros::adi::DigitalOut piston('e');
pros::adi::DigitalOut piston_score('c');
pros::adi::DigitalOut piston_park('b');

// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(0);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(0);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, 0);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0);





// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(
    drivetrain,          // drivetrain
    lateral_controller,  // lateral PID settings
    angular_controller,  // angular PID settings
    sensors              // odometry sensors
);




// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
	imu.tare_heading();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void movePID(int target, int maxSpeed, int timeout) {
    // PID constants — tune for your robot
    double kP = 1.0;
    double kD = 1.0;

    int error = 0;
    int prevError = 0;
    int derivative = 0;
    int measuredValue = 0;
    int motorOut = 0;

    // Reset motor encoder
    LeftFront.tare_position();

    int startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        // Get current position
        measuredValue = LeftFront.get_position();

        // Calculate error and derivative
        error = target - measuredValue;
        derivative = error - prevError;

        // Basic PD formula
        motorOut = (kP * error) + (kD * derivative);

        // Limit motor output
        if (motorOut > maxSpeed) motorOut = maxSpeed;
        if (motorOut < -maxSpeed) motorOut = -maxSpeed;

        // Move drivetrain motors (example for 4-motor tank drive)
        left_mg.move(motorOut);
        right_mg.move(motorOut);
    

        prevError = error;

        // Exit if within ±5 of target
        if (abs(error) <= 5) break;

        pros::delay(20);
    }

    // Stop motors at the end
    LeftFront.move(0);
    RightFront.move(0);
    LeftBack.move(0);
    RightBack.move(0);
}
void turnPID(int target, int maxSpeed, int timeout) {
    // PID constants — tune for your robot
    double kP = 1.0;
    double kD = 1.0;

    int error = 0;
    int prevError = 0;
    int derivative = 0;
    int measuredValue = 0;
    int motorOut = 0;

    // Reset motor encoder
    

    int startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        // Get current position
        measuredValue = imu.get_heading();

        // Calculate error and derivative
        error = target - measuredValue;
        derivative = error - prevError;

        // Basic PD formula
        motorOut = (kP * error) + (kD * derivative);

        // Limit motor output
        if (motorOut > maxSpeed) motorOut = maxSpeed;
        if (motorOut < -maxSpeed) motorOut = -maxSpeed;

        // Move drivetrain motors (example for 4-motor tank drive)
        left_mg.move(motorOut);
        right_mg.move(-motorOut);
    

        prevError = error;

        // Exit if within ±5 of target
        if (abs(error) <= 5) break;

        pros::delay(20);
    }

    // Stop motors at the end
    LeftFront.move(0);
    RightFront.move(0);
    LeftBack.move(0);
    RightBack.move(0);
} 

void brake(){
	left_mg.move(0);
	right_mg.move(0);
}
void intake(int Intake1, int Intake2, int Intake_score, int Intake_poop){
	intake1.move(Intake1);
	intake2.move(Intake2);
	intake_score.move(Intake_score);
	
}
void move(int speed, int time){
	left_mg.move(-speed);
	right_mg.move(speed);
	pros::delay(time);
	brake();
}
void turn(int dir, int speed, int time){
	left_mg.move(speed*dir);
	right_mg.move(speed*dir);
	pros::delay(time);
	brake();
}
void turn_new(int degree){
	left_mg.move(75);
	right_mg.move(75);
	while (true){
		if (imu.get_heading() == -degree){
			left_mg.move(0);
			right_mg.move(0);
			break;
		}
		pros::delay(20);
	}

}
void autonomous() { 
	bool right = true;
	if (right){
		

       move(50, 700); //forward and backwards
		pros::delay(400);
		turn(-1,50,150);//turn to blocks
		pros::delay(400);
		intake(80,80,0,0);//intake block
		move(40,900);
		pros::delay(700);
		intake(0,0,0,0);// stop intake
		pros::delay(700);
		move(-45,400);//move back
		turn(1,50,382);
		pros::delay(100);
		move(40,800);//move toward goal
		pros::delay(200);
		intake(-95,-95,-95,-95);//outtake
		pros::delay(200);
		intake(80,80,0,0);//intake block for clogging
		pros::delay(200);
		intake(-95,-95,-95,-95);//outtake
		pros::delay(2500);
		intake(0,0,0, 0);//stop intake
		move(-60,1200);//moves back long move
		piston.set_value(1);//pistion down
		turn(-1,40,1300);//turn to matchloader
		pros::delay(100);
		move(60,400);//moving to loading zone
		pros::delay(100);
		move(60,400);
		intake(100,100,100,100);//intake block
		pros::delay(500);
		move(40,50);//small move
		intake(0,0,0, 0);
	 
	}
	
		

	}


	




/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	
	


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		//dir and turn have swapped
		int dir = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);    // Gets amount forward/backward from left joystick
		int turn = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);  // Gets the turn left/right from right joystick
		left_mg.move(0.5*dir + turn);                      // Sets left motor voltage
		right_mg.move(0.5*dir - turn); 
	                                                             // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
		
		
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake(127,127,0,0);
			
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			piston.set_value(!piston_val);
			piston_val = !piston_val;
			pros::delay(500);
		 
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake(127,127,127,127);
		}	

		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake(-127,-127,-127,-127);
			
		}else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake(-90,127,127,127);
        }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			intake(90,-127,127,127);
        }    
            
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			piston_score.set_value(!piston_score_val);
			piston_score_val = !piston_score_val;
		    pros::delay(500);
		
       
        }
            
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			piston_park.set_value(!piston_park_val);
			piston_park_val = !piston_park_val;
		    pros::delay(500);
		
        }
		else{
			intake(0,0,0,0);	
		}
}
}

	
	
