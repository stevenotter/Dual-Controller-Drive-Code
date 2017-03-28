package org.usfirst.frc.team5586.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

/**
* Pipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class Pipeline implements VisionPipeline {

	//Outputs
	private Mat cvFlipOutput = new Mat();

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	@Override	public void process(Mat source0) {
		// Step CV_flip0:
		Mat cvFlipSrc = source0;
		FlipCode cvFlipFlipcode = FlipCode.X_AXIS;
		cvFlip(cvFlipSrc, cvFlipFlipcode, cvFlipOutput);

	}

	/**
	 * This method is a generated getter for the output of a CV_flip.
	 * @return Mat output from CV_flip.
	 */
	public Mat cvFlipOutput() {
		return cvFlipOutput;
	}


	/**
	 * Code used for CV_flip. 
	 * Per OpenCV spec 0 -> flip on X axis.
	 * >0 -> flip on Y axis.
	 * <0 -> flip on both axes.
	 */
	public enum FlipCode {
		X_AXIS(0),
		Y_AXIS(1),
		BOTH_AXES(-1);
		public final int value;
		FlipCode(int value) {
			this.value = value;
		}
	}	
	
	/**
	 * Flips an image along X, Y or both axes.
	 * @param src Image to flip.
	 * @param flipcode FlipCode of which direction to flip.
	 * @param dst flipped version of the Image.
	 */
	private void cvFlip(Mat src, FlipCode flipcode, Mat dst) {
		Core.flip(src, dst, flipcode.value);
	}




}

