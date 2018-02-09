/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2980.robot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix. motorcontrol.can.*;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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
	private static final double kAngleSetpoint1 = 0;
	private static final double kAngleSetpoint2 = -90;
	private static final double kP = 0.1; // propotional turning constant
	private static double move = 0;
	private static double velocity = 0;
	private static boolean step1 = false;
	private static boolean step2 = false;
	private static boolean ranAuto = false;
	private static String autoSelect = "";
	private static double power = 0;
	
	/*
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	private Pipeline myPipeline = new Pipeline();
	
	private final Object imgLock = new Object();
	*/
	
	WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(8);
	WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(6);
	WPI_TalonSRX backRightSlave = new WPI_TalonSRX(9);
	WPI_TalonSRX backLeftSlave = new WPI_TalonSRX(7);	

	Encoder encoder = new Encoder(0,1,true,EncodingType.k4X);
	AnalogInput dial = new AnalogInput(0);
	
	WPI_TalonSRX intakeLeft = new WPI_TalonSRX(2);
	WPI_TalonSRX intakeRight = new WPI_TalonSRX(3);

	WPI_TalonSRX Winch = new WPI_TalonSRX(4);

	WPI_TalonSRX lifter = new WPI_TalonSRX(5);
	
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	BuiltInAccelerometer accel = new BuiltInAccelerometer();

	Solenoid Grippy = new Solenoid(1);
	Solenoid Pushy = new Solenoid(0);
	
	Spark LED = new Spark(1);
	
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
    	
    	
    	CameraServer.getInstance().startAutomaticCapture();
    	/*
    	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	            }
	        }
	    });
	    visionThread.start();
	    */
		gyro.reset();
		gyro.calibrate();
		gyro.reset();
		encoder.setDistancePerPulse(6*3.1415/64.0*1.452);
		encoder.setSamplesToAverage(15);
		//encoder.setSamplesToAverage(5);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		move = 0;
		velocity = 0;
		m_timer.reset();
		gyro.reset();
		encoder.reset();
		m_timer.start();
		autoSelect = "";
		if(dial.getAverageVoltage()<2.5)
			autoSelect = "drive";
			
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		//double currentDistance = m_ultrasonic.getValue() * 0.125;
		//double currentSpeed = (currentDistance) * 0.05;
		double turningValue = 0;

		//dist = encoder.get()/12; //in ft
		/*
		velocity = encoder.getRate()/64;
		move += (0.3 - velocity)*0.5;
		if(move>1)
			move = 1;
		if(move<0)
			move = 0;
		*/
		if(autoSelect.equals("drive"))
		{
			if (m_timer.get() < 15.0 && encoder.get()<375 && !step1) { //set to 8.73 m for end of hall
				turningValue = (kAngleSetpoint1-gyro.getAngle()) * kP;
				if(turningValue>0.75)
					turningValue = 0.75;
				else if(turningValue<-0.75)
					turningValue = -0.75;
				if(m_timer.get()<1)
					turningValue = 0;
				m_robotDrive.arcadeDrive(0.65, -turningValue); // drive forwards half speed
			} 
			else if(!step1 && m_timer.get() < 15.0)
			{
				do
				{
					step1 = true;
					turningValue = (kAngleSetpoint2-gyro.getAngle()) * 0.017;
					if(turningValue>0.5)
						turningValue = 0.5;
					else if(turningValue<-0.5)
						turningValue = -0.5;
					if(turningValue>-0.25&&turningValue<=0)
						turningValue = -0.25;
					else if(turningValue<0.25&&turningValue>0)
						turningValue = 0.25;
					m_robotDrive.arcadeDrive(0, -turningValue);
					//dist = encoder.get()/12;
					encoder.reset();
					
			    	SmartDashboard.putNumber("Gyro", gyro.getAngle());
			    	SmartDashboard.putNumber("Drive", frontLeftDrive.get());
			    	//SmartDashboard.putNumber("Accel(g)", accel.getX());
			    	SmartDashboard.putNumber("move", move);
			    	SmartDashboard.putNumber("Velocity", velocity);
			    	SmartDashboard.putNumber("Distance(inches)", encoder.get());
			    	SmartDashboard.putNumber("Turn", turningValue);
			    	SmartDashboard.putNumber("Encoder", encoder.get());
			    	SmartDashboard.putNumber("Timer", m_timer.get());
			    	
				} while(gyro.getAngle()<-92 || gyro.getAngle()>-88);
			}
			else if(!step2&&encoder.get()<5)
			{
				do
				{
					step2 = true;
					turningValue = (kAngleSetpoint2-gyro.getAngle()) * kP;
					m_robotDrive.arcadeDrive(0.65, -turningValue);
				} while(encoder.get()<130);
			}
			
			else {
				m_robotDrive.stopMotor(); // stop robot
				ranAuto = true;
				//m_robotDrive.arcadeDrive(0, 0);
			}
		}
		
    	SmartDashboard.putNumber("Gyro", gyro.getAngle());
    	SmartDashboard.putNumber("Drive", frontLeftDrive.get());
    	//SmartDashboard.putNumber("Accel(g)", accel.getX());
    	SmartDashboard.putNumber("move", move);
    	SmartDashboard.putNumber("Velocity", velocity);
    	SmartDashboard.putNumber("Distance(ft)", encoder.get());
    	SmartDashboard.putNumber("Turn", turningValue);
    	SmartDashboard.putNumber("Encoder", encoder.get());
    	SmartDashboard.putNumber("Timer", m_timer.get());
    	SmartDashboard.putNumber("dial", dial.getAverageVoltage());
    	
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		move = 0;
		velocity = 0;
		//dist = 0;
		m_timer.reset();
		gyro.reset();
		encoder.reset();
		m_timer.start();
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		//Driving
		double forward = m_Stick.getY(); // logitech gampad left X, positive is forward
    	double turn = m_Stick.getX()*0.65; //logitech gampad right X, positive means turn right
    	
    	power += (forward-power)*0.5;
    	
    	m_robotDrive.arcadeDrive(-power, turn);
    	
    	//Pneumatics
    	
    	Grippy.set(m_Stick.getRawButton(1));
    	Pushy.set(m_Stick.getRawButton(2));
    	
    	//Lifter
    	
    	if(m_Stick.getRawButton(5)) {
    		lifter.set(1.0);
    	}
    	else if(m_Stick.getRawButton(7)) {
    		lifter.set(-0.5);
    	}
    	else {
    		lifter.set(0);
    	}
    	
    	//Intake

    	if(m_Stick.getRawButton(3)) {
    		intakeRight.set(1);
    	}
    	else if (m_Stick.getRawButton(4)) {
    		intakeRight.set(-1);
    	}
    	
    	SmartDashboard.putNumber("Gyro", gyro.getAngle());
    	SmartDashboard.putNumber("Accel", accel.getX());
    	SmartDashboard.putNumber("Encoder", encoder.get());
    	SmartDashboard.putNumber("move", move);
    	SmartDashboard.putNumber("Velocity", velocity);
    	SmartDashboard.putNumber("Distance(inches)", encoder.get());
    	SmartDashboard.putNumber("dial", dial.getAverageVoltage());
    	SmartDashboard.putNumber("drive", power);
    	//Encoder ticks = (360 / circumference) * Distance to travel
    	
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
}
