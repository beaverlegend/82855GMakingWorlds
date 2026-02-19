	#include "main.h"
	#include "config.h"
	#include "lemlib/api.hpp"
	#include <random>
	#include "pros/screen.hpp"
	#include "device/mockimu.h"


	

	/**
	 * A callback function for LLEMU's center button.
	 *
	 * When this callback is fired, it will toggle line 2 of the LCD text between
	 * "I was pressed!" and nothing.
	 */

	bool tonguePress = false;
	bool wingLift = false;
	bool intakeLift = false;
	bool indexing = true;
	

	double BOT_WIDTH = 18;
	double BOT_HEIGHT = 18;

	//mcl stuff
	struct Particle {
		double x;
		double y;
		double theta;
		double weight;
	};
	int numParticles = 200;
	std::vector<Particle> particles;
	std::vector<Particle> particles_snapshot;
	pros::Mutex particles_mutex;


	//current, delta, previous x, y values & heading
	double curx, dx, prex;
	double cury, dy, prey;
	double theta;

	double get_dist(pros::Distance &sensor) {
		return sensor.get() / 25.4;
	}

	//random distributions
	double sigma_x, sigma_y; //adds noise
	int seed = (int)(std::hash<std::string>{}("82855gonnagonnabegolden"));
	std::mt19937 gen(seed);
	double gauss(double mean=0, double stddev=1) {
		std::normal_distribution<double> dist(mean, stddev);
		return dist(gen);
	}
	double uniform(double minimum = -72.0, double maximum=72.0){
		std::uniform_real_distribution<double> dist(minimum, maximum);
		return dist(gen);
	}

	double adjust_angle(double theta, double add) {
		theta += add;
		while (theta >= 360.0) theta -= 360.0;
		while (theta <    0.0) theta += 360.0;
		return theta;
	}
	double inline_raycast(double x, double y, double theta){
		double hor;
		double ver;
		double angle;
		if(0<=theta&&theta<90){
			hor=72.0-x;
			ver=72.0-y;
			angle=theta;
		}
		else if(90<=theta&&theta<=180){
			hor=72.0-x;
			ver=72.0+y;
			angle=180-theta;
		}
		else if(180<theta&&theta<=270){
			hor=72.0+x;
			ver=72.0+y;
			angle=adjust_angle(theta, 180);
		}
		else{
			hor=72.0+x;
			ver=72.0-y;
			angle=360-theta;
		}
		if(angle==0)return ver;
		if(angle==90)return hor;
		return std::min(ver/cos(angle*M_PI/180), hor/sin(angle*M_PI/180));
	}



	void on_center_button()
	{
		static bool pressed = false;
		pressed = !pressed;
		if (pressed)
		{
			pros::lcd::set_text(2, "I was pressed!");
		}
		else
		{
			pros::lcd::clear_line(2);
		}
	}

	/**
	 * Runs initialization code. This occurs as soon as the program is started.
	 *
	 * All other competition modes are blocked by initialize; it is recommended
	 * to keep execution time for this mode under a few seconds.
	 */
	double scale = 20.6/24.0;
	lemlib::Drivetrain drivetrain(&left_mg,					  // left motor group
								&right_mg,				  // right motor group
								11,						  // 12 inch track width
								lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
								450, 						  // drivetrain rpm is 450
								2							  // horizontal drift is 2 (for now)
	);
	lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, -scale*lemlib::Omniwheel::NEW_325,0);
	lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
								nullptr,				  // vertical tracking wheel 2, set to nullptr as we are using IMEs
								nullptr,				  // horizontal tracking wheel 1
								nullptr,				  // horizontal tracking wheel 2, set to nullptr as we don't have a second one
								&imu					  // inertial sensor
	);

	// lateral PID controller
	lemlib::ControllerSettings lateral_controller(8,   // proportional gain (kP)
												0,   // integral gain (kI)
												31.7,  // derivative gain (kD)
												0,   // anti windup
												0,   // small error range, in inches
												100, // small error range timeout, in milliseconds
												3,   // large error range, in inches
												500, // large error range timeout, in milliseconds
												15   // maximum acceleration (slew)
	); 

	// angular PID controller
	lemlib::ControllerSettings angular_controller(9.0, // proportional gain (kP)3.2  5.8
												0,   // integral gain (kI)
												39,  // derivative gain (kD)28 39.5
												0,   // anti windup
												0,   // small error range, in degrees
												0,   // small error range timeout, in milliseconds
												0,   // large error range, in degrees
												0,   // large error range timeout, in milliseconds
												0	   // maximum acceleration (slew)
	);

	lemlib::ExpoDriveCurve
		steer_curve(3,	  // joystick deadband out of 127
					10,	  // minimum output where drivetrain will move out of 127
					1.019 // expo curve gain0
		);

	lemlib::ExpoDriveCurve
		throttle_curve(3,	 // joystick deadband out of 127
					10,	 // minimum output where drivetrain will move out of 127
					1.019 // expo curve gain
		);

	lemlib::Chassis chassis(drivetrain,			// drivetrain settings
							lateral_controller, // lateral PID settings
							angular_controller, // angular PID settings
							sensors				// odometry sensors
												// &throttle_curve, &steer_curve
	);

	/**
	 * Runs while the robot is in the disabled state of Field Management System or
	 * the VEX Competition Switch, following either autonomous or opcontrol. When
	 * the robot is enabled, this task will exit.
	 */
	/**S
	 * Runs initialization code. This occurs as soon as the program is started.
	 *
	 * All other competition modes are blocked by initialize; it is recommended
	 * to keep execution time for this mode under a few seconds.
	 */

	double init = -10000.00;
	void mclInitialize(){
		// init=true;
		// prex = chassis.getPose().x;
		// prey = chassis.getPose().y;
		// theta = chassis.getPose().theta;
		// //initialize particles with a random 
		// for(int i = 0 ; i < numParticles; i++){
		// 	Particle p = {uniform(), uniform(), theta, 1.0/numParticles};
		// 	particles.push_back(p);
		// 	particles_snapshot.push_back(p);
		// 	init = p.x;
		// }
		// sigma_x=0.7; sigma_y=0.7;

	}
	void mclLoop(){

		// //edit distances
		// curx = chassis.getPose().x;
		// cury = chassis.getPose().y;
		// theta = chassis.getPose().theta;

		// dx=curx-prex;
		// dy=cury-prey;

		// for(auto& p : particles){
		// 	p.x+=dx + gauss()*sigma_x; //add some random noise ?
		// 	p.y+=dy + gauss()*sigma_x;
		// 	p.theta=theta;
		// 	//resample if outside of boundaries
		// 	if(p.x<-72.0||p.x>72.0||p.y<-72.0||p.y>72.0){
		// 		p.x=uniform();
		// 		p.y=uniform();
		// 	}
		// }

		
		// double sum = 0;
		// double maxError = 5;
		// for(auto& p : particles){
		// 	double logW = 0.0;
		// 	double measurement;
		// 	double expected;
		// 	double error;
		// 	double sigma = 0.7; //sensor noise
		// 	//front
		// 	measurement = get_dist(dist_front)+BOT_WIDTH/2;
		// 	expected = inline_raycast(p.x, p.y, p.theta);
		// 	error=(abs(measurement - expected) > maxError)? maxError : measurement - expected;
		// 	logW+=-0.5*pow(error, 2) / (sigma*sigma);
		// 	//back
		// 	measurement = get_dist(dist_back)+BOT_WIDTH/2;
		// 	expected = inline_raycast(p.x, p.y, adjust_angle(p.theta, 180));
		// 	error=(abs(measurement - expected) > maxError)? maxError : measurement - expected;
		// 	logW+=-0.5*pow(error, 2) / (sigma*sigma);
		// 	//left
		// 	measurement = get_dist(dist_left)+BOT_HEIGHT/2;
		// 	expected = inline_raycast(p.x, p.y, adjust_angle(p.theta, 270));
		// 	error=(abs(measurement - expected) > maxError)? maxError : measurement - expected;
		// 	logW+=-0.5*pow(error, 2) / (sigma*sigma);
		// 	//right
		// 	measurement = get_dist(dist_right)+BOT_HEIGHT/2;
		// 	expected = inline_raycast(p.x, p.y, adjust_angle(p.theta, 90));
		// 	error=(abs(measurement - expected) > maxError)? maxError : measurement - expected;
		// 	logW+=-0.5*pow(error, 2) / (sigma*sigma);

		// 	p.weight = exp(logW);
		// 	sum+=p.weight;
		// }
		// //normalise the weights
		// for (auto &p : particles) p.weight /= sum;

		// double distanceMoved = sqrt(dx*dx + dy*dy);
		// bool doResample = (distanceMoved > 0.3); // threshold in same units as chassis pose

		// if (true) { //doResample
		// 	std::vector<Particle> newParticles;
		// 	int N = particles.size();
		// 	double c = particles[0].weight;
		// 	int i = 0;

		// 	for (int m = 0; m < N; m++) {
		// 		double U = (m + uniform(0.0, 1.0)) / N; // random in [m/N, (m+1)/N)
		// 		while (U > c) {
		// 			i++;
		// 			i = std::min(i, N-1);
		// 			c += particles[i].weight;
		// 		}
		// 		newParticles.push_back(particles[i]);
		// 	}
		// 	particles = newParticles;
			
		// 	// reset weights
		// 	for (auto &p : particles) p.weight = 1.0 / N;
		// 	// set heading to IMU
		// 	for (auto &p : particles) p.theta = theta;
		// }
		// double roughSigma = 0.1; // small variations
		// for (auto &p : particles) {
		// 	p.x += gauss() * roughSigma;
		// 	p.y += gauss() * roughSigma;
		// }

		// double xEst = 0.0;
		// double yEst = 0.0;
		// for (auto &p : particles) {
		// 	xEst += p.weight * p.x;
		// 	yEst += p.weight * p.y;
		// }

		// particles_mutex.take(TIMEOUT_MAX);
		// particles_snapshot = particles;
		// particles_mutex.give();

		// prex = curx;
		// prey = cury;

		// chassis.setPose(xEst, yEst, theta);



	}


	
	// --- screen / field mapping ---

	const int SCREEN_WIDTH = 480;
	const int SCREEN_HEIGHT = 272;

	// Field drawing area on the Brain screen
	const int FIELD_SIZE = 200;                      // 200x200 pixel square
	const int FIELD_X = SCREEN_WIDTH - FIELD_SIZE - 10; // right side with 10px margin
	const int FIELD_Y = 10;                          // top margin

	// world coords: x,y in [-72, 72] inches -> screen pixels
	int worldToScreenX(double x) {
		// map [-72, 72] -> [0, FIELD_SIZE]
		double t = (x + 72.0) / 144.0; // 0..1
		return FIELD_X + static_cast<int>(t * FIELD_SIZE);
	}

	int worldToScreenY(double y) {
		// map [-72, 72] -> [FIELD_SIZE, 0] (so +y is "up" on the screen)
		double t = (y + 72.0) / 144.0; // 0..1
		return FIELD_Y + FIELD_SIZE - static_cast<int>(t * FIELD_SIZE);
	}
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	void initialize() {
		
		pros::lcd::initialize();
		
		chassis.calibrate();
		
		pros::Task screen_task([&]()
							{
		
			while (true) {
				
			
				// std::vector<Particle> local;
				// particles_mutex.take(TIMEOUT_MAX);
				// local = particles_snapshot;
				// particles_mutex.give();
				// // --- Text debug on the left side ---
				// // Clear text area with black
				// pros::screen::set_pen(0x000000); // black
				// pros::screen::fill_rect(0, 0, 200, 80);

				// auto pose = chassis.getPose();

				// Draw text in white
				pros::screen::set_pen(0xFFFFFF); // white
				// master.print(0, 0, "Th:%.1f", chassis.getPose().theta);
				master.print(0,0, "XYT: %.1f %.1f %.1f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
				pros::screen::print(TEXT_MEDIUM, 0, "X: %.2f", chassis.getPose().x);
				pros::screen::print(TEXT_MEDIUM, 1, "Y: %.2f", chassis.getPose().y);
				pros::screen::print(TEXT_MEDIUM, 2, "Th: %.2f", chassis.getPose().theta);
				pros::screen::print(TEXT_MEDIUM, 3, "wing: %d", wingLift);
				pros::screen::print(TEXT_MEDIUM, 4, "scraper: %d", tonguePress);
				pros::screen::print(TEXT_MEDIUM, 5, "intake: %d", intakeLift);
				pros::screen::print(TEXT_MEDIUM, 6, "index: %d", indexing);
				
				
				// pros::screen::print(TEXT_MEDIUM, 3, "first: %.2f", (local.empty())? -10000.00 : local[0].x);
				// pros::screen::print(TEXT_MEDIUM, 4, "init: %d", particles_snapshot.size());

				// // --- Clear field area ---
				// pros::screen::set_pen(0x000000); // black
				// pros::screen::fill_rect(
				//     FIELD_X, FIELD_Y,
				//     FIELD_X + FIELD_SIZE,
				//     FIELD_Y + FIELD_SIZE
				// );

				// // Draw field border in white
				// pros::screen::set_pen(0xFFFFFF); // white
				// pros::screen::draw_rect(
				//     FIELD_X, FIELD_Y,
				//     FIELD_X + FIELD_SIZE,
				//     FIELD_Y + FIELD_SIZE
				// );

				// pros::screen::set_pen(0x0000FF);
				// for (const auto &p : local) {
				// 	int sx = worldToScreenX(p.x);
				// 	int sy = worldToScreenY(p.y);
				// 	if (sx >= FIELD_X && sx < FIELD_X + FIELD_SIZE &&
				// 		sy >= FIELD_Y && sy < FIELD_Y + FIELD_SIZE) {
				// 		pros::screen::fill_circle(sx, sy, 1);
				// 	}
				// }

				// // --- Draw estimated robot pose as a small red circle ---
				// int rx = worldToScreenX(pose.x);
				// int ry = worldToScreenY(pose.y);
				// pros::screen::set_pen(0xFF0000); // red
				// pros::screen::fill_circle(rx, ry, 4); // radius 3px

				pros::delay(20);
			} });
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
//reset postion COde
// 1. Helper to get a "Clean" distance using a Normal Distribution approach
// We take multiple samples to find the stable mean.
double getNormalDistributedDistance(pros::Distance& sensor) {
    double sum = 0;
    int samples = 10;
    int valid_samples = 0;

    for(int i = 0; i < samples; i++) {
        double dist = sensor.get_distance();
        
        // V5 sensors return 9999 or 0 when they fail/glitch
        if(dist > 0 && dist < 3000) { 
            sum += dist;
            valid_samples++;
        }
        pros::delay(10); // Small delay so we don't read the exact same photon bounce
    }

    if (valid_samples == 0) return -1; // Return error if sensor is blocked/disconnected
    
    // Mean of the distribution converted to inches
    return (sum / valid_samples) / 25.4;
}

// 2. The Reset Function
void resetPoseWithSensors() {
    // These constants should be measured from the center of your robot 
    // to the face of the sensor.
    const double FRONT_OFFSET = 4.0; 
    const double SIDE_OFFSET = 3.5;  

    // Get our "cleaned" values
    double frontDist = getNormalDistributedDistance(frontDistance);
    double sideDist = getNormalDistributedDistance(leftDistance);

    // If sensors are glitching, exit the function to avoid teleporting the robot to (0,0)
    if (frontDist == -1 || sideDist == -1) return;

    // Get current heading to handle the trig
    // LemLib uses degrees; we need Radians for C++ math
    double thetaRad = chassis.getPose().theta * M_PI / 180.0;

    // Calculate unit vectors based on current heading
    double Fy = cos(thetaRad); // Forward component
    double Rx = cos(thetaRad); // Right/Side component (assuming sensor faces Left)

    /* LOGIC:
       If frontDist is measured against the North wall (Y = 72):
       True Y = 72 - (Distance + Offset) * cos(theta)
       
       If sideDist is measured against the West wall (X = -72):
       True X = -72 + (Distance + Offset) * sin(theta) 
    */
    
    double trueY = 72.0 - (frontDist + FRONT_OFFSET) * Fy;
    double trueX = -72.0 + (sideDist + SIDE_OFFSET) * sin(thetaRad);

    // Apply the new coordinates to LemLib while keeping the current IMU heading
    chassis.setPose(trueX, trueY, chassis.getPose().theta);
}

	void toggleWing() // lift or lower intake
	{
		wingLift = !wingLift;
	}
	void adjustWing()
	{
		wings.set_value(wingLift);
	}

	void toggleTongue() // lift or lower tongue
	{
		tonguePress = !tonguePress;
	}
	void adjustTongue()
	{
		tongue.set_value(tonguePress);
	}

	void toggleIntake()
	{
		intakeLift=!intakeLift;
	}
	void adjustIndex()
	{
		indexer.set_value(!indexing);
	}
	void adjustIntake()
	{
		intakeFinal.set_value(intakeLift);
	}

	// intake FUNCTIONS
	// int speed = 0;
	void intakeHighgoal()
	{
		// speed = 127;
		Intake_High_mg.move(-127);
		// intakeLift = true;
		// adjustIntake();
	}

	void intakeMiddlegoal()
	{
		Intake_High_mg.move(-70);
		// intakeLift = false;
		// indexing=false;
		// adjustIntake();
	}

	void IntakeSlowReverse(){
		// speed = -100;
		Intake_High_mg.move(100);
	}
	void IntakeReverse()
	{
		// speed=-127;
		Intake_High_mg.move(127);
	}

	void intakeStop()
	{
		// speed = 0;
		Intake_High_mg.move(0);
	}
	// void moveIntake()
	// {
	// 	Intake_High_mg.move(speed);
	// }
	// void antijam(){

	// }
	//AUTON CODES

	void pidforwardTune()
	{
		while (true)
		{
			
			chassis.moveToPoint(0,24, 4000);
			pros::delay(4000);
			chassis.moveToPoint(0,0, 4000, {.forwards = false});
			pros::delay(4000);
			chassis.moveToPoint(0,12, 4000);
			pros::delay(4000);
			chassis.moveToPoint(0,0, 4000, {.forwards = false});
			pros::delay(4000);
		}
	}
	void pidTurnTune(){
		master.clear();
		chassis.setPose(0, 0, 0);
		while (true)
		{	
			
			chassis.turnToHeading(90, 3000);
			pros::delay(4000);
			chassis.turnToHeading(0, 3000);
			pros::delay(4000);
			chassis.turnToHeading(180, 3000);
			pros::delay(4000);
			chassis.turnToHeading(0, 3000);
			pros::delay(4000);
			// chassis.moveToPoint(0,48, 4000);
			// pros::delay(4000);
			// chassis.moveToPoint(0,0, 4000, {.forwards = false});
			// pros::delay(4000);
			
		}
	}
void DistanceSensorTest(){
	chassis.moveToPoint(0, 60, 1000);

	pros::delay(200);

	resetPoseWithSensors();

	chassis.moveToPoint(30, 30, 1000);

}
	void redBottom(){
		// adjustWing();
		//start
		chassis.setPose(-49, -17, 100);
		pros::delay(100);
		//intake first 3
		// intakeIndex();
		chassis.moveToPose(-15, -26, 135, 2000, {.lead = 0.3, .maxSpeed = 80});
		pros::delay(700);
		tonguePress=true;
		adjustTongue();

		// chassis.moveToPoint(-15, -16, 2000, {.maxSpeed = 40});
		// intakeStop();
		pros::delay(750);
		
		
		//score
		//chassis.turnToHeading(45, 1000);
		chassis.moveToPose(-10.5, -13, 45, 2000, {.lead = 0.2, .maxSpeed = 70});
		toggleTongue();
		adjustTongue();	
		pros::delay(400);
		intakeStop();
		pros::delay(800);
		IntakeSlowReverse();
		pros::delay(2500);
		// chassis.moveToPose(-3, -10, 45, 1000, {.lead = 0.1, .maxSpeed = 10});
		// pros::delay(4000);
		intakeStop();
		pros::delay(500);

		// back
		chassis.moveToPoint(-43, -47, 1000, {.forwards = false, .maxSpeed = 80});
		pros::delay(1000);
		chassis.turnToHeading(270, 1000);
		pros::delay(1000); //super long delay because I need to fix everything before this
		tonguePress = true;
		adjustTongue();
		pros::delay(500);
		intakeHighgoal();
		chassis.moveToPoint(-66, -47, 1020, {.forwards = true, .maxSpeed = 65});
		pros::delay(500);
		intakeStop();
		pros::delay(200);
		// intakeIndex();
		intakeLift = true;
		chassis.moveToPoint(-14, -47, 2000, {.forwards = false, .maxSpeed = 60});
		wingLift = false;
		adjustWing();
		adjustIntake();
		pros::delay(1000);
		intakeHighgoal();
		

		//matchload
	}

	void justinsawp(){
		chassis.setPose(-48, -15, 180);

		//matchload 1
		chassis.moveToPoint(-48, -47, 1000);
		pros::delay(1000);
		intakeHighgoal();
		chassis.turnToHeading(270, 800, {.maxSpeed = 90});
		tonguePress=true;
		adjustTongue();
		pros::delay(800);
		//intakeHighgoal();
		chassis.moveToPoint(-64, -47, 920, {.maxSpeed=80});
		intakeLift=true;
		adjustIntake();
		pros::delay(920);

		//score 4 balls
		chassis.moveToPoint(-28, -47, 1100, {.forwards=false, .maxSpeed=95});
		pros::delay(950);
		indexing=false;
		adjustIndex();
		pros::delay(1300);

		//setup position
		tonguePress=false;
		adjustTongue();
		indexing=true;
		adjustIndex();
		chassis.moveToPoint(-42, -45, 500);
		intakeLift=false;
		adjustIntake();


		//intake 3 balls
		pros::delay(500);
		chassis.turnToHeading(40, 500);
		pros::delay(500);
		chassis.moveToPoint(-21, -21, 500, {.earlyExitRange = 3});
		chassis.moveToPoint(-21, -21, 500, {.maxSpeed  = 70});
		pros::delay(700);
		chassis.turnToHeading(0, 600);
		pros::delay(600);

		//intake 3 balls
		chassis.moveToPoint(-22, 25, 1300);
		pros::delay(900);
		tonguePress=true;
		adjustTongue();

		pros::delay(300);
		chassis.turnToHeading(310, 600);
		pros::delay(600);

		//score some balls
		chassis.moveToPoint(-12, 16, 700, {.forwards=false});
		intakeMiddlegoal();
		pros::delay(700);
		indexing=false;
		adjustIndex();
		pros::delay(1000);
		indexing=true;
		adjustIndex();
		intakeHighgoal();

		//matchload 3 balls
		chassis.moveToPoint(-47, 47.5, 900);
		pros::delay(900);
		chassis.turnToHeading(270, 800);
		// indexing=true;
		// adjustIndex();
		pros::delay(800);
		chassis.moveToPoint(-65, 47.5, 920, {.maxSpeed=80});
		pros::delay(920);
		intakeLift=true;
		adjustIntake();

		//score rest of balls
		chassis.moveToPoint(-28, 47.5, 1000, {.forwards=false});
		pros::delay(900);
		indexing=false;
		adjustIndex();
	}

	void justinRS() {
		chassis.setPose(-48, -15, 180);

		//matchload 1
		chassis.moveToPoint(-48, -46, 1000);
		pros::delay(1000);
		intakeHighgoal();
		chassis.turnToHeading(270, 800, {.maxSpeed = 90});
		tonguePress=true;
		adjustTongue();
		pros::delay(800);
		chassis.moveToPoint(-64, -46, 980, {.maxSpeed=80});
		intakeLift=true;
		adjustIntake();
		pros::delay(980);

		//score 4 balls
		chassis.moveToPoint(-28, -46, 1100, {.forwards=false});
		pros::delay(1100);
		indexing=false;
		adjustIndex();
		pros::delay(3000);

		//setup position
		tonguePress=false;
		adjustTongue();
		indexing=true;
		adjustIndex();
		chassis.moveToPoint(-42, -46, 500);
		intakeLift=false;
		adjustIntake();


		//intake 3 balls
		pros::delay(500);
		chassis.turnToHeading(40, 500);
		pros::delay(500);
		chassis.moveToPoint(-25, -21, 500, {.earlyExitRange = 3});
		chassis.moveToPoint(-25, -21, 500, {.maxSpeed  = 70});
		pros::delay(1000);

		chassis.moveToPoint(-10.5, -12, 600, {.maxSpeed=60});
		pros::delay(800);
		chassis.turnToHeading(34, 800);
		pros::delay(800);
		IntakeSlowReverse();

	}

	void justinLS() {
		chassis.setPose(-50, 15, 80);

		//grab 3
		intakeHighgoal();
		chassis.moveToPoint(-22, 22, 1000);
		pros::delay(600);
		tonguePress=true;
		adjustTongue();
		pros::delay(700);

		//score
		chassis.turnToHeading(315, 800);
		pros::delay(800);
		chassis.moveToPoint(-13, 9, 800, {.forwards=false});
		pros::delay(800);
		intakeMiddlegoal();
		indexing=false;
		adjustIndex();
		pros::delay(1000);
		indexing=true;
		adjustIndex();
		intakeHighgoal();

		//matchload 3 balls
		chassis.moveToPoint(-47, 46, 1000);
		pros::delay(1000);
		chassis.turnToHeading(270, 800);
		pros::delay(1000);
		// tonguePress=true;
		// adjustTongue();
		pros::delay(500);
		chassis.moveToPoint(-64, 43.2, 920, {.maxSpeed=80});
		pros::delay(920);
		intakeLift=true;
		adjustIntake();

		//score rest of balls
		chassis.moveToPoint(-30, 43.2, 1000, {.forwards=false});
		pros::delay(900);
		indexing=false;
		adjustIndex();
		pros::delay(3000);
		tonguePress=false;
		adjustTongue();

		//wing push
		// chassis.moveToPoint(-25, 47, 800);
		// pros::delay(800);
		// chassis.turnToHeading(110, 800);
		// pros::delay(800);
		// chassis.moveToPoint(-22, 36, 1000);
		// pros::delay(1000);
		// chassis.turnToHeading(90, 800);
		// pros::delay(800);
		// chassis.moveToPoint(-9, 37, 1000);



	}

	void skills() {
		chassis.setPose(-48, -15, 180);

		//matchload 1
		chassis.moveToPoint(-48, -45, 1000);
		pros::delay(1000);
		intakeHighgoal();
		chassis.turnToHeading(270, 800, {.maxSpeed = 90});
		tonguePress=true;
		adjustTongue();
		pros::delay(800);
		intakeHighgoal();
		chassis.moveToPoint(-64, -45, 900, {.maxSpeed=80});
		intakeLift=true;
		adjustIntake();
		pros::delay(2000);

		//score 7 balls
		chassis.moveToPoint(-28, -45, 1100, {.forwards=false});
		pros::delay(1100);
		indexing=false;
		adjustIndex();
		pros::delay(4000);

		//setup position
		tonguePress=false;
		adjustTongue();
		indexing=true;
		adjustIndex();
		chassis.moveToPoint(-42, -45, 500);

		//intake 3 balls
		pros::delay(500);
		chassis.turnToHeading(40, 500);
		pros::delay(500);
		chassis.moveToPoint(-20, -21, 500, {.earlyExitRange = 3});
		chassis.moveToPoint(-20, -21, 500, {.maxSpeed  = 70});
		pros::delay(1000);
		chassis.turnToHeading(0, 600);
		pros::delay(600);

		//intake 3 balls
		chassis.moveToPoint(-22, 25, 1300);

		pros::delay(1300);
		chassis.turnToHeading(310, 800);
		pros::delay(1000);

		chassis.moveToPoint(-47, 50, 1000);
		pros::delay(1000);
		chassis.turnToHeading(270, 800);
		pros::delay(1000);


		chassis.moveToPoint(-30, 50, 1000, {.forwards=false});
		pros::delay(1000);
		indexing=false;
		adjustIndex();
		pros::delay(4000);

		indexing=true;
		adjustIndex();
		pros::delay(1000);
		tonguePress=true;
		adjustTongue();
		pros::delay(500);
		chassis.moveToPoint(-67, 50, 1000);
		pros::delay(3000);

		chassis.moveToPoint(-30, 50, 1000, {.forwards=false});
		pros::delay(1000);
		indexing=false;
		adjustIndex();
		pros::delay(4000);
	}

	void LeftSide(){
		chassis.setPose(-49,16,80);
		pros::delay(100);
		intakeHighgoal();
		chassis.moveToPoint(-26,23,500, {.maxSpeed = 80});
		pros::delay(500);
		chassis.moveToPose(-11,11,135,1000, {.lead = 0.2, .maxSpeed = 80 });


		
		
		
	}

	void autonomous() {
		//chassis.moveToPoint(0,8,1000);
		// pidTurnTune();
		//justinsawp();
		//justinLS();
	justinRS();
		//skills();
		// redBottom();
	}




	//pneumatics



	// void IntakeStop(){
	// 	Intake_High_mg.move(0);
	// 	Intake_Middle_mg.move(0);
	// }
	// void IntakeScore(){
	// 	Intake_Shooter.move(127);
	// 	Intake_Shooter_Beaver.move(60);
	// }
	void opcontrol() {
		
		// mclInitialize();
		chassis.setPose(0, 0, 0);
		

		while (true) {
			// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
			//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
			//                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

			// Arcade control scheme
			int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
			int turn = (int)(0.8*master.get_analog(ANALOG_RIGHT_X));  // Gets the turn left/right from right joystick
			
			
			left_mg.move(dir + turn);                      // Sets left motor voltage
			right_mg.move(dir - turn);
			
						
			// Sets right motor voltage
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
				// intakeIndex();
				intakeHighgoal();
				indexing=true;

			}
			//reverse high goal
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
				IntakeReverse();
			}
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
				intakeMiddlegoal();
				intakeLift=false;
				indexing=false;
			}
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
				intakeHighgoal();
				intakeLift = true;
				indexing=false;
				
			}
			else{
				intakeStop();
			}
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT )){
				toggleTongue();
			}
			// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT )){
			// 	toggleIntake();
			// }
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
				toggleWing();
			}
			// else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			// 	IntakeScore();
			// }
			
			adjustTongue();
			adjustWing();
			adjustIndex();
			adjustIntake();
			// moveIntake();
			// mclLoop();
			pros::delay(50);  
										// Run for 20 ms then update
		}

	}