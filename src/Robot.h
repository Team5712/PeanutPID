/*
 * Robot.h
 *
 *  Created on: May 7, 2018
 *      Author: robotics
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "WPILib.h"

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
//#include <opencv2/core.hpp>

#include <iostream>
#include <string>
#include <IterativeRobot.h>

#include "ctre/Phoenix.h"

class Robot : public frc::IterativeRobot {
public:
	Robot();

	// frc methods
	void RobotInit();
	void TeleopInit();
	void TeleopPeriodic();

	void AutonomousPeriodic();

	// custom methods
	void TankDrive(double, double);

	void resetEncoders();
	double getLeftPosition();
	double getRightPosition();

private:


	CameraServer cam;

	struct pid_values {
		// change to 512
		double rpm = 461;
		int ticksPerRevolution = 125;

		double accel = 461;
		double vel = 461;

		double P = 0.0;
		double I = 0.0;
		double D = 0.0;
		double F = 2.22190889370932754880694143167028;
		double timeout = 0.005;
	} pid;

	void initPID();

	Timer timer_encoder;

	WPI_TalonSRX *motor_l;
	WPI_TalonSRX *motor_r;

	DifferentialDrive *drive;

	Joystick *joystick_l;
	Joystick *joystick_r;

};


#endif /* SRC_ROBOT_H_ */
