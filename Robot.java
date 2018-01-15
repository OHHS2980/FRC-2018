/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2980.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix. motorcontrol.can.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Ultrasonic;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(2);
	WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(4);
	WPI_TalonSRX backRightSlave = new WPI_TalonSRX(3);
	WPI_TalonSRX backLeftSlave = new WPI_TalonSRX(5);
	/*
	WPI_TalonSRX intakeLeft = new WPI_TalonSRX(6);
	WPI_TalonSRX intakeRight = new WPI_TalonSRX(7);

	WPI_TalonSRX winch1 = new WPI_TalonSRX(8);
	WPI_TalonSRX winch2 = new WPI_TalonSRX(9);

	WPI_TalonSRX lifterLeft = new WPI_TalonSRX(10);
	WPI_TalonSRX lifterRight = new WPI_TalonSRX(11);
*/
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	BuiltInAccelerometer accel = new BuiltInAccelerometer();
	//private AnalogInput m_ultrasonic = new AnalogInput(1);
	
	private DifferentialDrive m_robotDrive
			= new DifferentialDrive(frontLeftDrive, frontRightDrive);
	private Joystick m_Stick = new Joystick(0);
	private Timer m_timer = new Timer();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
    	backRightSlave.follow(frontRightDrive);
    	backLeftSlave.follow(frontLeftDrive);
		CameraServer.getInstance().startAutomaticCapture();
		gyro.reset();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		//double currentDistance = m_ultrasonic.getValue() * 0.125;
		//double currentSpeed = (currentDistance) * 0.05;

		if (m_timer.get() < 15.0) {
			m_robotDrive.arcadeDrive(1, 0.0); // drive forwards half speed
		} else {
			m_robotDrive.stopMotor(); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		double forward = m_Stick.getY(); // logitech gampad left X, positive is forward
    	double turn = m_Stick.getX(); //logitech gampad right X, positive means turn right
    	m_robotDrive.arcadeDrive(forward, -turn);
    	SmartDashboard.putNumber("Gyro", gyro.getAngle());
    	SmartDashboard.putNumber("Accel", accel.getX());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
