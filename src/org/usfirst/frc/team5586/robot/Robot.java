
package org.usfirst.frc.team5586.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends IterativeRobot {
	// Define all variables used for Autonomous selection.
	final String defaultAuto = "Blank";
	final String customAuto1 = "Vision Centering X + Y";
	final String customAuto2 = "Vision Control Stick";
	final String customAuto3 = "Move Forward";
	String autoSelected;
	SendableChooser<String> autoChooser = new SendableChooser<String>();
	
	// Define all variables used for teleOp selection.
	final String mode1 = "Xbox Driving";
	final String mode2 = "Flight-Stick Driving";
	String teleOpSelected;
	SendableChooser<String> teleOpChooser = new SendableChooser<String>();
	Timer matchTimer;
	
	// Create the motor controllers used in the RobotDrive and link them together for the drive.
    Victor mFrontLeft = new Victor(0);
    Victor mBackLeft = new Victor(1);
    Victor mFrontRight = new Victor(2);
    Victor mBackRight = new Victor(3);
    RobotDrive drive = new RobotDrive(mFrontLeft, mBackLeft, mFrontRight, mBackRight);
    
    // Create other actuators to be used later.
    Spark intake = new Spark(7);
    Spark winch = new Spark(6);
    Servo feeder = new Servo(4);
    
    // Create the controller objects.
    XboxController xbox  = new XboxController(0);
    Joystick flight = new Joystick(0);
    
    // Create the gyro object.
    ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    
    // Create all global variables used in vision processing later.
    private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	private double centerX = 0.0;
	private double centerY = 0.0;
	private final Object imgLock = new Object();
	double timer = 0;
	double moveX = 0;
	double moveY = 0;
	double moveZ = 0;
    
	// ROBOT INIT - RUNS WHEN THE ROBOT TURNS ON.
	
    public void robotInit() {
    	// Calibrate Gyro when the robot turns on.
        gyro.calibrate(); 
        
        // Invert motors on one side for the mecanum drive to function correctly.
        mFrontRight.setInverted(true);
        mBackRight.setInverted(true);
        
        // Start the camera streaming to the dashboard and set the resolution.
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        
        // Create an instance of the vision pipeline to be run, and define what variables we want to pull out of it.
        VisionThread visionThread = new VisionThread(camera, new Pipeline(), pipeline -> {
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (imgLock) {
                    centerX = r.x + (r.width / 2);
                    centerY = r.y + (r.height / 2);
                }
            }
        });
        
        // Run the vision pipeline.
        visionThread.start();
        
        // Add options to the SmartDashboard for Auto.
        autoChooser.addDefault("Blank Auto", defaultAuto);
        autoChooser.addObject("Shooting Assist", customAuto1);
		autoChooser.addObject("Driving Assist", customAuto2);
		SmartDashboard.putData("Auto choices", autoChooser);
    }
    
    // AUTONOUMOUS INIT - RUNS ONCE WHEN THE AUTONOMOUS PERIOD BEGINS.
    
    public void autonomousInit() {
    	// Define which Auto was selected to a variable and print which one is selected to the console.
    	autoSelected = autoChooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		
		// Set the timer to 0 at the start of the Auto period.
		timer = 0;
    }
    
    // AUTONOMOUS PERIODIC - RUNS THROUGHOUT THE AUTONOMOUS PERIOD.
    
	public void autonomousPeriodic() {
		// Define variables to be used in auto later and increment the timer.
		timer += 0.002;
		SmartDashboard.putNumber("Time in Auto", timer);
		double centerX;
		double centerY;
		
		// Switch statement to select the correct Autonomous code.
		switch (autoSelected) {
		case customAuto3:
			
			break;
		case customAuto2:
			// Define variables for later use in this Auto.
			boolean foundX = false;
			boolean foundY = false;
			double t = 3;
			
			// Call the synchronized variable "centerX".
			synchronized (imgLock) {
				centerX = this.centerX;
				centerY = this.centerY;
			}
			
			// Calculate how far off the camera is from the center of the found object.
			double offX = (centerX - (IMG_WIDTH / 2));
			double offY = (centerY - (IMG_HEIGHT /2));
			
			// Create a tolerance for the centering of the camera on the X axis, and calculate motor speed based on % error.
			if(offX < t && offX > -t) {
				foundX = true;
			} else { 
				foundX = false;
				moveX = offX * 0.005;
				moveX += timer * 0.05;
			}
			
			// Create a tolerance for the centering of the camera on the Y axis, and calculate motor speed based on % error.
			if(offY < t && offY > -t) {
				foundY = true;
			} else { 
				foundY = false;
				moveY = offY * 0.005;
				moveY += timer * 0.05;
			}
			
			// Publish the % error and whether or not the correct X and Y values have been found.
			SmartDashboard.putNumber("Off X", offX);
			SmartDashboard.putNumber("Off Y", offY);
			SmartDashboard.putBoolean("Found X", foundX);
			SmartDashboard.putBoolean("Found Y", foundY);
			
			// Set the robot to drive according to all of the calculated variables.
			drive.mecanumDrive_Cartesian(moveX, -moveY, 0, gyro.getAngle());
			break;
		case customAuto1:
			// Call the synchronized variable "centerX".
			synchronized (imgLock) {
				centerX = this.centerX;
			}
			
			// Calculate the magnitude of the movement along the X axis during this Auto.
			double move = (centerX - (IMG_WIDTH / 2)) * 0.005;
			
			// Set the drive to move according to the "move" variable and stabilized with gyro.
			drive.mecanumDrive_Cartesian(move, 0, 0, gyro.getAngle());
			break;
		case defaultAuto:
		default:
			// Set the default Auto to keep the robot still.
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			break;
		}
	}

	// TELEOP PERIODIC - RUNS THROUGHOUT THE TELEOP PERIOD.
	
    public void teleopPeriodic() {
    	// Define variables to be used later in teleOp
        double xSpeed;
        double ySpeed;
        double zSpeed;
        
        // Publish information to the SmartDashboard.
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        
        // Define which teleOp was selected to a variable
        teleOpSelected = teleOpChooser.getSelected();
    	
        // Switch statement to select the correct Autonomous code.
        switch (teleOpSelected) {
        case mode1:
        	// Collect controller input, and set the "xSpeed" variable for forward and backward movement.
        	if(xbox.getRawAxis(4) < .1 && xbox.getRawAxis(4) > -.1) {
        		xSpeed = 0;
        	} else {
        		xSpeed = xbox.getRawAxis(4);
        	}
    	
        	// Collect controller input, and set the "ySpeed" variable for side to side movement.
        	if(xbox.getRawAxis(5) < .1 && xbox.getRawAxis(5) > -.1) {
        		ySpeed = 0;
        	} else {
        		ySpeed = xbox.getRawAxis(5);
        	}
    	
    		// Collect controller input, and set the "zSpeed" variable for rotation.
    		if(xbox.getRawAxis(0) < .2 && xbox.getRawAxis(0) > -.2) {
    			zSpeed = 0;
    		} else {
    			zSpeed = xbox.getRawAxis(0);
    		}
    	
    		// Reset the gyro on the push of a button.
    		if(xbox.getYButton() == true) {
    			gyro.reset();
    		}
    		
    		// Move servo in relation to a button press.
    		if(xbox.getAButton() == true) {
    			feeder.setAngle(100);
    		} else if(xbox.getAButton() == false) {
    			feeder.setAngle(0);
    		}
    	
        	// Set the motor speed to the calculated variables and add the vector given by the gyro.
    		drive.mecanumDrive_Cartesian(xSpeed, ySpeed, zSpeed, gyro.getAngle());
    		break;
        case mode2:
        	// Collect controller input, and set the "xSpeed" variable for forward and backward movement.
        	if(flight.getRawAxis(0) < .1 && flight.getRawAxis(0) > -.1) {
        		xSpeed = 0;
        	} else {
        		xSpeed = flight.getRawAxis(0);
        	}
    	
        	// Collect controller input, and set the "ySpeed" variable for side to side movement.
        	if(flight.getRawAxis(1) < .1 && flight.getRawAxis(1) > -.1) {
        		ySpeed = 0;
        	} else {
        		ySpeed = flight.getRawAxis(1);
        	}
    	
    		// Collect controller input, and set the "zSpeed" variable for rotation.
    		if(flight.getRawAxis(2) < .2 && flight.getRawAxis(2) > -.2) {
    			zSpeed = 0;
    		} else {
    			zSpeed = flight.getRawAxis(2);
    		}
    	
    		// Reset the gyro on the push of a button.
    		if(flight.getRawButton(6) == true) {
    			gyro.reset();
    		}
    	
        	// Set the motor speed to the calculated variables and add the vector given by the gyro.
    		drive.mecanumDrive_Cartesian(xSpeed, ySpeed, zSpeed, gyro.getAngle());
        	break;
        }
        
    }

    // TEST PERIODIC - RUNS WHILE THE ROBOT IS IN TEST MODE.
    
    public void testPeriodic() {
    
    }
    
}
