
package org.usfirst.frc.team5586.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
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
	final String defaultAuto = "Blank";
	final String customAuto1 = "Drive Forward";
	final String customAuto2 = "Vision Control Stick";
	String autoSelected;
	SendableChooser<String> autoChooser = new SendableChooser<String>();
	
	// Define all variables used for teleOp selection.
	final String defaultmode = "Dual Driving";
	final String mode1 = "Xbox Driving";
	final String mode2 = "Flight-Stick Driving";
	String teleOpSelected;
	SendableChooser<String> teleOpChooser = new SendableChooser<String>();
	Timer myTimer = new Timer();
	
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
    
    // Create all global variables used in vision processing later.
    private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	private double centerX1 = 0.0;
	private double centerY1 = 0.0;
	private double centerX2 = 0.0;
	private double centerY2 = 0.0;
	private double X1 = 0.0;
	private double Y1 = 0.0;
	private double X2 = 0.0;
	private double Y2 = 0.0;
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
        camera.setFPS(20);
        
        // Create an instance of the vision pipeline to be run, and define what variables we want to pull out of it.
      /*  VisionThread visionThread = new VisionThread(camera, new Pipeline(), pipeline -> {
            if (pipeline.filterContoursOutput().size() < 2) {
                Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
                synchronized (imgLock) {
                    centerX1 = r1.x + (r1.width / 2);
                    centerY1 = r1.y + (r1.height / 2);
                    centerX2 = r2.x + (r2.width / 2);
                    centerY2 = r2.y + (r2.height / 2);
                }
            }
        }); */
        
        // Run the vision pipeline.
        // visionThread.start();
        
        // Add options to the SmartDashboard for Auto.
        autoChooser.addDefault("Blank Auto", defaultAuto);
        autoChooser.addObject("Drive Forward", customAuto1);
		autoChooser.addObject("Vision Control Stick", customAuto2);
		SmartDashboard.putData("Auto choices", autoChooser);
		
		// Add options to the Smart Dashboard for TeleOp.
		teleOpChooser.addDefault("Dual Driving", defaultAuto);
    	teleOpChooser.addObject("Xbox Driving", mode1);
    	teleOpChooser.addObject("Flight-Stick Driving", mode2);
    	SmartDashboard.putData("teleOp Choices", teleOpChooser);
    }
    
    // AUTONOMOUS INIT - RUNS ONCE WHEN THE AUTONOMOUS PERIOD BEGINS.
    
    public void autonomousInit() {
    	// Define which Auto was selected to a variable and print which one is selected to the console.
    	autoSelected = autoChooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		
		// Start the timer.
		myTimer.start();
		
		// Set the timer to 0 at the start of the Auto period.
		myTimer.reset();
		
    }
    
    // AUTONOMOUS PERIODIC - RUNS THROUGHOUT THE AUTONOMOUS PERIOD.
    
	public void autonomousPeriodic() {
		// Switch statement to select the correct Autonomous code.
		switch (autoSelected) {
		case customAuto2:
			// Define variables for later use in this Auto.
			boolean foundX = false;
			boolean foundY = false;
			double t = 3;
						
			// Call the synchronized variable "centerX".
			synchronized (imgLock) {
				X1 = this.centerX1;
				Y1 = this.centerY1;
				X2 = this.centerX2;
				Y2 = this.centerY2;
			}
			
			double middleX = (X1 + ((X2 - X1)/2));
			double middleY = ((Y1 + Y2)/2);
			
						
			// Calculate how far off the camera is from the center of the found object.
			double offX = (middleX - (IMG_WIDTH / 2));
			double offY = (middleY - (IMG_HEIGHT /2));
						
			// Create a tolerance for the centering of the camera on the X axis, and calculate motor speed based on % error.
			if(offX < t && offX > -t) {
				foundX = true;
			} else { 
				foundX = false;
				moveX = offX * 0.00125;
				//moveX += timer * 0.05;
			}
						
			// Create a tolerance for the centering of the camera on the Y axis, and calculate motor speed based on % error.
			if(offY < t && offY > -t) {
				foundY = true;
			} else { 
				foundY = false;
				moveY = offY * 0.00125;
				//moveY += timer * 0.05;
			}
						
			// Set the robot to drive according to all of the calculated variables.
			drive.mecanumDrive_Cartesian(moveX, moveY, 0, gyro.getAngle());
			break;
		
		case customAuto1:
			if (myTimer.get() <= 3) {
				drive.mecanumDrive_Cartesian(0, -.60, 0, 0);
			} else {
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				myTimer.stop();
			}
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
        		xSpeed = (xbox.getRawAxis(4))*(xbox.getRawAxis(4));
        	}
    	
        	// Collect controller input, and set the "ySpeed" variable for side to side movement.
        	if(xbox.getRawAxis(5) < .1 && xbox.getRawAxis(5) > -.1) {
        		ySpeed = 0;
        	} else {
        		ySpeed = (xbox.getRawAxis(5))*(xbox.getRawAxis(5));
        	}
    	
    		// Collect controller input, and set the "zSpeed" variable for rotation.
    		if(xbox.getRawAxis(0) < .2 && xbox.getRawAxis(0) > -.2) {
    			zSpeed = 0;
    		} else {
    			zSpeed = (xbox.getRawAxis(0))*(xbox.getRawAxis(0));
    		}
    	
    		// Reset the gyro on the push of a button.
    		if(xbox.getYButton() == true) {
    			gyro.reset();
    		}
    		
    		// Move servo in relation to a button press.
    		if(xbox.getAButton() == true) {
    			feeder.setAngle(109);
    		} else if(xbox.getAButton() == false) {
    			feeder.setAngle(180);
    		}
    		
    		// Start intake on button press.
    		if(xbox.getTriggerAxis(GenericHID.Hand.kLeft) > .1) {
    			intake.set(1);
    		} else {
    			intake.set(0);
    		}
    		
    		// Start ball shooter on button press.
    		if(xbox.getTriggerAxis(GenericHID.Hand.kRight) > .1) {
    			shooter.set(1);
    		} else {
    			shooter.set(0);
    		}
    		
    		// Start the climber on button press.
    		if(xbox.getBumper(GenericHID.Hand.kLeft) == true) {
    			winch.set(1);
    		} else {
    			winch.set(0);
    		}
    	
        	// Set the motor speed to the calculated variables and add the vector given by the gyro.
    		drive.mecanumDrive_Cartesian(xSpeed, ySpeed, zSpeed, 0);
    		break;
        case mode2:
        	// Collect controller input, and set the "xSpeed" variable for forward and backward movement.
        	if(flight.getRawAxis(0) > .1) {
        		xSpeed = (flight.getRawAxis(0))*(flight.getRawAxis(0));
        	} else if (flight.getRawAxis(0) < -.1){
        		xSpeed = -((flight.getRawAxis(0))*(flight.getRawAxis(0)));
        	} else {
        		xSpeed = 0;
        	}
    	
        	// Collect controller input, and set the "ySpeed" variable for side to side movement.
        	if(flight.getRawAxis(1) > .1) {
        		ySpeed = (flight.getRawAxis(1))*(flight.getRawAxis(1));
        	} else if (flight.getRawAxis(1) < -.1){
        		ySpeed = -((flight.getRawAxis(1))*(flight.getRawAxis(1)));
        	} else {
        		ySpeed = 0;
        	}
    	
    		// Collect controller input, and set the "zSpeed" variable for rotation.
        	if(flight.getRawAxis(2) > .1) {
        		zSpeed = (flight.getRawAxis(2))*(flight.getRawAxis(2));
        	} else if (flight.getRawAxis(2) < -.1){
        		zSpeed = -((flight.getRawAxis(2))*(flight.getRawAxis(2)));
        	} else {
        		zSpeed = 0;
        	}
    		
    		// Start the shooter.
    		if(flight.getRawButton(1) == true) {
    			shooter.set(1);
    		}
    		
    		// Start the intake.
    		if(flight.getRawButton(2) == true) {
    			intake.set(1);
    		}
    		
    		// Move the feeder to open position.
    		if(flight.getRawButton(3) == true) {
    			feeder.setAngle(109);
    		} else {
    			feeder.setAngle(180);
    		}
    		
    		// Start the winch.
    		if(flight.getRawButton(4) == true) {
    			winch.set(1);
    		}
    		
    		// Reset the gyro on the push of a button.
    		if(flight.getRawButton(6) == true) {
    			gyro.reset();
    		}
    	
        	// Set the motor speed to the calculated variables and add the vector given by the gyro.
    		drive.mecanumDrive_Cartesian(xSpeed, ySpeed, zSpeed, gyro.getAngle());
        	break;
        default:
        	// Collect controller input, and set the "xSpeed" variable for forward and backward movement.
        	if(flight.getRawAxis(0) > .1) {
        		xSpeed = (flight.getRawAxis(0))*(flight.getRawAxis(0));
        	} else if (flight.getRawAxis(0) < -.1){
        		xSpeed = -((flight.getRawAxis(0))*(flight.getRawAxis(0)));
        	} else {
        		xSpeed = 0;
        	}
    	
        	// Collect controller input, and set the "ySpeed" variable for side to side movement.
        	if(flight.getRawAxis(1) > .1) {
        		ySpeed = (flight.getRawAxis(1))*(flight.getRawAxis(1));
        	} else if (flight.getRawAxis(1) < -.1){
        		ySpeed = -((flight.getRawAxis(1))*(flight.getRawAxis(1)));
        	} else {
        		ySpeed = 0;
        	}
    	
    		// Collect controller input, and set the "zSpeed" variable for rotation.
        	if(flight.getRawAxis(2) > .1) {
        		zSpeed = (flight.getRawAxis(2))*(flight.getRawAxis(2));
        	} else if (flight.getRawAxis(2) < -.1){
        		zSpeed = -((flight.getRawAxis(2))*(flight.getRawAxis(2)));
        	} else {
        		zSpeed = 0;
        	}
    		
    		if(xbox.getAButton() == true) {
    			feeder.setAngle(109);
    		} else if(xbox.getAButton() == false) {
    			feeder.setAngle(180);
    		}
    		
    		// Start intake on button press.
    		if(xbox.getTriggerAxis(GenericHID.Hand.kLeft) > .1) {
    			intake.set(1);
    		} else {
    			intake.set(0);
    		}
    		
    		// Start ball shooter on button press.
    		if(xbox.getTriggerAxis(GenericHID.Hand.kRight) > .1) {
    			shooter.set(1);
    		} else {
    			shooter.set(0);
    		}
    		
    		// Start the winch.
    		if(flight.getRawButton(4) == true) {
    			winch.set(1);
    		} else {
    			winch.set(0);
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
