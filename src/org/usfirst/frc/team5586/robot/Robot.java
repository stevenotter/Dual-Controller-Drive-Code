package org.usfirst.frc.team5586.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends IterativeRobot {
	// Define all variables used for Autonomous selection.
	final static String BLANK_AUTO = "Blank";
	String autoSelected;
	SendableChooser<String> autoChooser = new SendableChooser<String>();
	
	// Create the motor controllers used in the RobotDrive and link them together for the drive.
    Victor mFrontLeft = new Victor(0);
    Victor mBackLeft = new Victor(1);
    Victor mFrontRight = new Victor(2);
    Victor mBackRight = new Victor(3);
    RobotDrive drive = new RobotDrive(mFrontLeft, mBackLeft, mFrontRight, mBackRight);
    
    // Create other actuators to be used later.
    Spark intake = new Spark(7);
    Spark winch = new Spark(6);
    Servo feeder = new Servo(5);
    VictorSP shooter = new VictorSP(4);
    
    // Create the controller objects.
    XboxController xbox  = new XboxController(0);
    Joystick flight = new Joystick(1);
    
    // Create the gyro object.
    ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    
    //Pneumatic actuator declarations
    Compressor compressor;
    Solenoid solenoidOpen = new Solenoid(0);
    Solenoid solenoidClose = new Solenoid(1);
    Solenoid solenoidExtend = new Solenoid(2);
    Solenoid solenoidRetract = new Solenoid(3);
    Solenoid solenoidUp = new Solenoid(4);
    Solenoid solenoidDown = new Solenoid(5);
    boolean armIsUp = true;
    
    // Create all global variables used later.
    private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	private static final double JOYSTICK_TOLERANCE = .1;
	double centerX1;
	double centerX2;
	Object imgLock = new Object();
	double totalCenter;
    boolean centerLock = false;
    
	// ROBOT INIT - RUNS WHEN THE ROBOT TURNS ON.
	
    public void robotInit() {
    	// Calibrate Gyro when the robot turns on.
        gyro.calibrate(); 
        
        // Invert motors on one side for the mecanum drive to function correctly.
        mFrontRight.setInverted(true);
        mBackRight.setInverted(true);
        
        // Start the compressor.
        // Add code back here
        
        // Start the camera streaming to the dashboard and set the resolution.
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        camera.setFPS(30);
        
        // Create an instance of the vision pipeline to be run, and define what variables we want to pull out of it.
//        VisionThread visionThread = new VisionThread(camera, new Pipeline(), pipeline -> {
//            if (pipeline.filterContoursOutput().size() < 2) {
//                Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
//                Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
//                synchronized (imgLock) {
//                    centerX1 = r1.x + (r1.width / 2);
//                    centerX2 = r2.x + (r2.width / 2);
//                    totalCenter = (((centerX2 - centerX1)/2) + centerX1);
//                }
//            }
//        });       
        
        // Run the vision pipeline.
//        visionThread.start();
        
        // Add options to the SmartDashboard for Auto.
        autoChooser.addDefault("Blank Auto", BLANK_AUTO);
		SmartDashboard.putData("Auto choices", autoChooser);
    }

	// AUTONOMOUS INIT - RUNS ONCE WHEN THE AUTONOMOUS PERIOD BEGINS.
    
    public void autonomousInit() {
    	// Define which Auto was selected to a variable and print which one is selected to the console.
    	autoSelected = autoChooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		
    }
    
    // AUTONOMOUS PERIODIC - RUNS THROUGHOUT THE AUTONOMOUS PERIOD.
    
	public void autonomousPeriodic() {

		// TODO Autonomous should probably do something
		
		// Switch statement to select the correct Autonomous code.
		switch (autoSelected) {
		case BLANK_AUTO:
			// Set the Auto to keep the robot still.
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			break;
		default:
			// Set the default Auto to keep the robot still.
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			break;
		}
		
	}

	// TELEOP PERIODIC - RUNS THROUGHOUT THE TELEOP PERIOD.
	
    public void teleopPeriodic() {
    	// Define variables to be used later in teleOp
        double xSpeed = getSpeed(flight.getRawAxis(0));
        double ySpeed = getSpeed(flight.getRawAxis(1));
        double zSpeed = getSpeed(flight.getRawAxis(2));
        double xDisplacement;
        synchronized(imgLock) {
        	xDisplacement = (totalCenter - (IMG_WIDTH/2)) * 0.05;
        }
        
        compressor = new Compressor(0);
        
        // Set the position of the solenoids on button press.
        solenoidOpen.set(xbox.getBumper(Hand.kLeft));;
		solenoidClose.set(!xbox.getBumper(Hand.kLeft));
		solenoidExtend.set(xbox.getBumper(Hand.kRight));
		solenoidRetract.set(!xbox.getBumper(Hand.kRight));
		
		// Use DPad to raise and lower arm, default up
		if(xbox.getPOV() == 180) {
			armIsUp = true;
		} else if(xbox.getPOV() == 0) {
			armIsUp = false;
		}
		solenoidUp.set(armIsUp);
		solenoidDown.set(!armIsUp);
		
		// Start the winch.
		if(flight.getRawButton(4) == true) {
			winch.set(1);
		} else {
			winch.set(0);
		}
		
		// Engage the robot to lock onto the X axis center.
		// TODO - We should be robot-centric when using centerLock
		if(flight.getRawButton(3)) {
			centerLock = true;
		} else {
			centerLock = false;
		}
		
		//Reset gyro at push of a button
		if(flight.getRawButton(6) == true) {
			gyro.reset();
		}
    	
		if (centerLock) {
			drive.mecanumDrive_Cartesian(xDisplacement, ySpeed, 0, gyro.getAngle());
		} else {
			drive.mecanumDrive_Cartesian(xSpeed, ySpeed, zSpeed, gyro.getAngle());
		}
    }
    
    private double getSpeed(double axis) {
    	double speed = 0;
    	
    	if(axis > JOYSTICK_TOLERANCE) {
    		speed = axis*axis;
    	} else if (axis < -JOYSTICK_TOLERANCE){
    		speed = -(axis*axis);
    	} else {
    		speed = 0;
    	}
    	
    	return speed;
    }
       
    // TEST PERIODIC - RUNS WHILE THE ROBOT IS IN TEST MODE.
    
    public void testPeriodic() {
    
    }
    
}
