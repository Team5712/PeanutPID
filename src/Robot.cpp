/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
using namespace std;

Robot::Robot() {

	joystick_l = new Joystick(0);
	joystick_r = new Joystick(1);

	motor_l = new WPI_TalonSRX(2);
	motor_r = new WPI_TalonSRX(1);

	drive = new DifferentialDrive(*motor_l, *motor_r);

	cam = CameraServer::GetInstance()->StartAutomaticCapture(0);

}

void Robot::initPID() {

	motor_l->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
			pid.timeout);
	motor_r->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
			pid.timeout);

	motor_l->SelectProfileSlot(0, 0);
	motor_r->SelectProfileSlot(0, 0);

	motor_l->Config_kP(0, pid.P, pid.timeout);
	motor_l->Config_kI(0, pid.I, pid.timeout);
	motor_l->Config_kD(0, pid.D, pid.timeout);
	motor_l->Config_kF(0, pid.F, pid.timeout);

	motor_r->Config_kP(0, pid.P, pid.timeout);
	motor_r->Config_kI(0, pid.I, pid.timeout);
	motor_r->Config_kD(0, pid.D, pid.timeout);
	motor_r->Config_kF(0, pid.F, pid.timeout);

	motor_l->ConfigMotionAcceleration(pid.accel, pid.timeout);
	motor_l->ConfigMotionCruiseVelocity(pid.vel, pid.timeout);

	motor_r->ConfigMotionAcceleration(pid.accel, pid.timeout);
	motor_r->ConfigMotionCruiseVelocity(pid.vel, pid.timeout);

	resetEncoders();
}

void Robot::RobotInit() {
	resetEncoders();
}

void Robot::TeleopInit() {
	resetEncoders();
}

void Robot::TeleopPeriodic() {

	cout << "l " << motor_l->GetSelectedSensorPosition(0) << endl;
	cout << "r " << motor_r->GetSelectedSensorPosition(0) << endl;

	// number of ticks per revolution * 10 revo * input
	double drive_l = 512 * 10 * joystick_l->GetRawAxis(1);
	double drive_r = 512 * 10 * joystick_r->GetRawAxis(1);

	if (joystick_l->GetRawButton(1) || joystick_r->GetRawButton(1)) {
		motor_l->Set(ControlMode::MotionMagic, drive_l);
		motor_r->Set(ControlMode::MotionMagic, drive_r);
	} else {
		TankDrive(joystick_l->GetRawAxis(1), joystick_r->GetRawAxis(1));
	}

	// 512 ticks / rev!!!

}

void Robot::TankDrive(double l, double r) {
	drive->TankDrive(l, r);
}

void Robot::AutonomousPeriodic() {
	motor_l->Set(ControlMode::MotionMagic, 2000);
	motor_r->Set(ControlMode::MotionMagic, 2000);
}

double Robot::getLeftPosition() {
	return motor_l->GetSelectedSensorPosition(0);
}

double Robot::getRightPosition() {
	return motor_r->GetSelectedSensorPosition(0);
}

void Robot::resetEncoders() {
	motor_l->SetSelectedSensorPosition(0, 0, 0);
	motor_r->SetSelectedSensorPosition(0, 0, 0);
}

START_ROBOT_CLASS(Robot)
