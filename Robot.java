/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2980.robot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix. motorcontrol.can.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private static final double kP = 0.2; // proportional turning constant
	private static double move = 0;
	private static double velocity = 0;
	private static boolean step1 = false;
	private static boolean step2 = false;
	private static boolean ranAuto = false;
	private static String autoSelect = "";
	private static double power = 0;
	private static boolean grippyState = false;
	private static boolean pushyState = false;
	private static boolean button1 = false;
	private static boolean button2 = false;
	private static boolean hasRunAuto =  false;
	private static String gameData = "";
	
	WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(8);
	WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(6);
	WPI_TalonSRX backRightSlave = new WPI_TalonSRX(9);
	WPI_TalonSRX backLeftSlave = new WPI_TalonSRX(7);	
	
	Encoder encoder = new Encoder(0,1,true,EncodingType.k4X);
	AnalogInput dial = new AnalogInput(0);
	
	WPI_TalonSRX intakeLeft = new WPI_TalonSRX(2);
	WPI_TalonSRX intakeRight = new WPI_TalonSRX(3);

	WPI_TalonSRX Winch = new WPI_TalonSRX(4);

	WPI_TalonSRX lifter1 = new WPI_TalonSRX(5);
	WPI_TalonSRX lifter2 = new WPI_TalonSRX(10);
	
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	BuiltInAccelerometer accel = new BuiltInAccelerometer();

	Solenoid grippy = new Solenoid(0);
	Solenoid pushy = new Solenoid(1);
	
	Spark LED = new Spark(0);
	
	//limit switch that returns true if the lifter is all the way extended
	//DigitalInput liftLimit = new DigitalInput(8);
	
	private DifferentialDrive m_robotDrive
			= new DifferentialDrive(frontLeftDrive, frontRightDrive);
	private Timer m_timer = new Timer();
	
	//JOYSTICK And Buttons
	
	private Joystick m_Stick = new Joystick(0);

	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		//Sets matching drive and intake motors to follow each other
		
    	backRightSlave.follow(frontRightDrive);
    	backLeftSlave.follow(frontLeftDrive);
    	intakeLeft.follow(intakeRight);
    	lifter2.follow(lifter1);
    	
    	LED.set(0.45);
    	    	
		gyro.reset();
		gyro.calibrate();
		gyro.reset();
		encoder.setDistancePerPulse(6*3.1415/64.0*2);
		encoder.setSamplesToAverage(15);
		//encoder.setSamplesToAverage(5);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		hasRunAuto = false;
		move = 0;
		velocity = 0;
		m_timer.reset();
		gyro.reset();
		encoder.reset();
		m_timer.start();
		if(dial == null||dial.getAverageVoltage()==0)
			autoSelect = "line";
		else if(dial.getAverageVoltage()<0.7)
			autoSelect = "scaleR";
		else if(dial.getAverageVoltage()<1.46)
			autoSelect = "switchR";
		else if(dial.getAverageVoltage()<3.3)
			autoSelect = "line";
		else if(dial.getAverageVoltage()<4)
			autoSelect = "switchL";
		else if(dial.getAverageVoltage()<5)
			autoSelect = "scaleL";
		else
			autoSelect = "line";
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		SmartDashboard.putString("autoSelect", autoSelect);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		double turningValue = 0;
		double timer = m_timer.get();
		if(!hasRunAuto) {
			if(autoSelect.equals("line") || gameData == "") {
				driveF(130, 150, 1, 0.5);
			}
			
			
			//RIGHT SIDE SWITCH
			if(autoSelect.equals("switchR")) {
				
				//makes the robot drive forward until it reaches a certain distance
				driveF(140, 190, 1, 0.67);
				
				//turns left
				while(m_timer.get()<14.7 && gyro.getAngle()>-88.5) {
					turningValue = (-90+gyro.getAngle()) * 0.012;
					turningValue = makeReasonable(turningValue, 0.60);
					m_robotDrive.arcadeDrive(0.0, -turningValue);
					dataOutput();
				}
				encoder.reset();
				
				
				
				//SWITCH ON OTHER SIDE (Drives more)
				if(gameData.charAt(0)=='L') {
					while(encoder.getDistance()<135 && m_timer.get()<14.7)
					{
						m_robotDrive.arcadeDrive(-0.67, 0);
					}
				}
				
				//SWITCH ON OUR SIDE (Drives less)
				else{
					while(encoder.getDistance()<33 && m_timer.get()<14.7)
					{
						m_robotDrive.arcadeDrive(-0.67, 0);
					}
				}
				
				
				
				//Turns(left) to face switch
				gyro.reset();
				while(m_timer.get()<14.7 && gyro.getAngle()>-88.5) {
					turningValue = (-90+gyro.getAngle()) * 0.012;
					turningValue = makeReasonable(turningValue, 0.60);
					m_robotDrive.arcadeDrive(0.0, -turningValue);
				}
				encoder.reset();
				
				
				m_robotDrive.arcadeDrive(0, 0);
				
				liftAndDrive(1.75,1);
				
				//move lifter down for X sec
				timer = m_timer.get();
				while(m_timer.get()-timer<1.25 && m_timer.get()<14.7) {
					pushy.set(true);
					lifter1.set(-0.5);
				}
				
				//intake wheels begin spinning and robot moves toward cube
				timer = m_timer.get();
				while(m_timer.get()-timer < 1.25 && m_timer.get() < 14.7)
				{
					intakeRight.set(-1);
					m_robotDrive.arcadeDrive(-0.38, 0.0);
			    	dataOutput();
				}
				
				//grab cube and suck it in for X sec (and drive forward)
				timer = m_timer.get();
				while(m_timer.get()-timer < 1 && m_timer.get() < 14.7)
				{
					intakeRight.set(-1);
					grippy.set(false);
					m_robotDrive.arcadeDrive(-0.25, 0.0);
			    	dataOutput();
				}
				
				
				//(repeat of above code)
				//lifts and spits cube
				
				// raises lifter for ? seconds
				timer = m_timer.get();
				while(m_timer.get()-timer < 2 && m_timer.get() < 14.7) {
					lifter1.set(1.0);
					//drive forward as lifter lifts
					//m_robotDrive.arcadeDrive(-0.5, 0.0);
				}
				lifter1.set(0);

				SmartDashboard.putNumber("Timer Diff", m_timer.get()-timer);
		    	dataOutput();	
		    	
		    	//push out before we eject the cube
		    	timer = m_timer.get();
		    	while(m_timer.get()-timer<0.5)
		    		pushy.set(true);
				
				
				//intake wheels eject cube for 1 second
				timer = m_timer.get();
				while(m_timer.get()-timer < 1 && m_timer.get() < 14.7)
				{
					intakeRight.set(1);
			    	dataOutput();
				}
			}
				
			
			if(autoSelect.equals("switchL"))
			{

				//makes the robot drive forward until it reaches a certain distance
				driveF(140, 190, 1, 0.67);
				
				//turns right
				while(m_timer.get() < 14.7 && gyro.getAngle() < 82.5) {
					turningValue = (90+gyro.getAngle()) * 0.012;
					turningValue = makeReasonable(turningValue, 0.60);
					m_robotDrive.arcadeDrive(0.0, -turningValue);
					dataOutput();
				}
				encoder.reset();
						
				//SWITCH ON OTHER SIDE (Drives more)
				if(gameData.charAt(0)=='R') {
					while(encoder.getDistance() < 135 && m_timer.get() < 14.7)
					{
						m_robotDrive.arcadeDrive(-0.67, 0);
					}
				}
				
				//SWITCH ON OUR SIDE (Drives less)
				else{
					while(encoder.getDistance() < 33 && m_timer.get() < 14.7)
					{
						m_robotDrive.arcadeDrive(-0.67, 0);
					}
				}
				
				//Turns(right) to face switch
				gyro.reset();
				while(m_timer.get() < 14.7 && gyro.getAngle() < 82.5) {
					turningValue = (90+gyro.getAngle()) * 0.012;
					turningValue = makeReasonable(turningValue, 0.60);
					m_robotDrive.arcadeDrive(0.0, -turningValue);
				}
				encoder.reset();
				
				
				m_robotDrive.arcadeDrive(0, 0);
				
				//lifts and drives forward
				liftAndDrive(1.75,1);
				
				//setting up to grab next cube
				grippy.set(true);
				pushy.set(true);

				//move lifter down for X sec
				timer = m_timer.get();
				while(m_timer.get()-timer < 1.25 && m_timer.get() < 14.7) {
					pushy.set(true);
					lifter1.set(-0.5);
				}
				
				//intake wheels begin spinning and robot moves toward cube
				timer = m_timer.get();
				while(m_timer.get()-timer < 1.25 && m_timer.get() < 14.7)
				{
					intakeRight.set(-1);
					m_robotDrive.arcadeDrive(-0.38, 0.0);
			    	dataOutput();
				}
				
				//grab cube and suck it in for X sec (and drive forward)
				timer = m_timer.get();
				while(m_timer.get()-timer < 1 && m_timer.get() < 14.7)
				{
					intakeRight.set(-1);
					grippy.set(false);
					m_robotDrive.arcadeDrive(-0.25, 0.0);
			    	dataOutput();
				}
				
				//lifts without driving forward
				liftAndDrive(2,0);
				
			}
			
		}
    	hasRunAuto = true;    	
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		power = 0;
		move = 0;
		m_timer.reset();
		m_timer.start();
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		//Driving
		double forward = m_Stick.getY(); // logitech gampad left X, positive is forward
    	double turn = m_Stick.getZ()*0.88; //logitech gampad right X, positive means turn right
    	
    	power += (forward-power)*0.85;
    	
    	m_robotDrive.arcadeDrive(-power, -turn);
    	
    	//Pneumatics
    	/*toggles states of grippy and pushy solenoids by checking if the status of the button 
    	is true when the previous state was false*/
    	if(m_Stick.getRawButton(1)&&!button1)
    	{
    		grippyState = !grippyState;
    	}
    	if(m_Stick.getRawButton(2)&&!button2)
    	{
    		pushyState = !pushyState;
    	}
    	//sets state of solenoids based on toggle
    	grippy.set(grippyState);
    	pushy.set(pushyState);
    	
    	//stores values of buttons for toggle system
    	button1 = m_Stick.getRawButton(1);
    	button2 = m_Stick.getRawButton(2);
 
    	
    	//lifters
    	//raise the lifter
    	if(m_Stick.getRawButton(5)) {
    		lifter1.set(1.0);
    	}
    	//lower the lifter
    	else if(m_Stick.getRawButton(7)) {
    		lifter1.set(-0.5);
    	}
    	else {
    		lifter1.stopMotor();
    	}
    	
    	//Intake
    	//draw into robot
    	if(m_Stick.getRawButton(4)) {
    		intakeRight.set(1);
    	}
    	//push out of robot
    	else if (m_Stick.getRawButton(3)) {
    		intakeRight.set(-1);
    	}
    	else {
    		intakeRight.stopMotor();
    	}
    	
    	if(m_Stick.getRawButton(6)) {
    		Winch.set(1); //climb up
    	}
    	else if (m_Stick.getRawButton(8)) {
    		Winch.set(-1); //descend
    	}
    	else {
    		Winch.set(0);
    	}
    	/*
    	if( m_timer.get() == 105 ) {
        	LED.set(-0.11);
    	}
    	else if (m_timer.get() > 107) {
    		LED.set(0.65);
    	}  
    	*/
    	/*
    	dataOutput();
    	SmartDashboard.putNumber("drive", power);
    	SmartDashboard.putBoolean("grippyState", grippyState);
    	SmartDashboard.putBoolean("pushyState", pushyState);
    	*/
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void dataOutput() {
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
    	SmartDashboard.putNumber("Drive", frontLeftDrive.get());
    	//SmartDashboard.putNumber("move", move);
    	SmartDashboard.putNumber("Velocity", velocity);
    	SmartDashboard.putNumber("Distance(in)", encoder.getDistance());
    	SmartDashboard.putNumber("Encoder", encoder.get());
    	SmartDashboard.putNumber("Timer", m_timer.get());
    	SmartDashboard.putNumber("dial", dial.getAverageVoltage());
    	//SmartDashboard.putBoolean("liftLimit", liftLimit.get());
	}
	//d is the value to be made reasonable, e is the absolute deviation from 0
	public double makeReasonable(double d, double e) {
		if(d>e)
			d = e;
		else if(d<-e)
			d = -e;
		return d;
	}

	private void driveF(double break1, double break2, double speed1, double speed2)
	{
		double turningValue;
		while(encoder.getDistance() < break2 && m_timer.get() < 14.7) {
			//keeps the robot driving straight
			turningValue = (0-gyro.getAngle()) * kP;
			turningValue = makeReasonable(turningValue, 0.5);
			if(m_timer.get()<0.5)
				turningValue = 0;
			if(encoder.getDistance()<break1)
				m_robotDrive.arcadeDrive(-speed1, -turningValue); // drive forwards
			else
				m_robotDrive.arcadeDrive(-speed2, -turningValue); // drive forwards
			grippy.set(false);
	    	SmartDashboard.putNumber("Turn", turningValue);
	    	dataOutput();
		}
	}
	private void liftAndDrive(double lTime, double dTime)
	{	
		m_robotDrive.arcadeDrive(0, 0);
		
		// raises lifter for 1.75 seconds (first lift)
		double timer = m_timer.get();
		while(m_timer.get()-timer < lTime && m_timer.get() < 14.7) {
			lifter1.set(1.0);
			//drive forward as lifter lifts
			if(m_timer.get()-timer<dTime)
				m_robotDrive.arcadeDrive(-0.3, 0.0);
		}
		lifter1.set(0);
		timer = m_timer.get();
    	while(m_timer.get()-timer<0.5)
    		pushy.set(true);
		
		//intake wheels eject cube for 1 second
		timer = m_timer.get();
		while(m_timer.get()-timer < 1 && m_timer.get() < 14.7)
		{
			intakeRight.set(1);
	    	dataOutput();
	    	pushy.set(true);
		}
		
		//setting up to grab next cube
		grippy.set(true);
		pushy.set(true);
	}
	
}