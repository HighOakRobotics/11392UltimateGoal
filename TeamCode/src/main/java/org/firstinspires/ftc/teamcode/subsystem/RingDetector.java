package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.ftc11392.sequoia.util.Clock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingDetector extends Subsystem {

	protected OpenCvCamera camera;
	protected Clock clock;

	protected final int ROWS = 640;
	protected final int COLS = 480;

	protected static final Scalar RED = new Scalar(255, 0, 0);
	protected static final Scalar GREEN = new Scalar(0, 255, 0);
	protected static final Scalar BLUE = new Scalar(0, 0, 255);

	//-------------------------------------------------------

	protected static int valBottom = -1;
	protected static int valTop = -1;

	protected static int numOfRings;

	protected static float rectHeight = 0.3f / 8f; //0.2f/8f;
	protected static float rectWidth = 1.2f / 8f; //0.84f/8f;

	protected static float offsetX = 1.75f / 8f;//changing this moves the two rects and the two circles left or right, range : (-2, 2) not inclusive
	protected static float offsetY = 1.3f / 8f; // move down 0f/8f;//changing this moves the two rects and circles up or down, range: (-4, 4) not inclusive
	//moves all rectangles right or left by amount. units are in ratio to monitor
	protected static float[] bottomPos = {4f / 8f + offsetX, 2.6f / 8f + offsetY};//0 = col, 1 = row
	protected static float[] topPos = {4f / 8f + offsetX, 2.2f / 8f + offsetY}; //0f/8f is the highest

	@Override
	public void initialize(HardwareMap hardwareMap) {
		clock = new Clock();
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
		camera.openCameraDevice();
		camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

		camera.setPipeline(new RingPipeline());
	}

	@Override
	public void initPeriodic() {

	}

	@Override
	public void start() {

	}

	@Override
	public void runPeriodic() {

	}

	@Override
	public void stop() {
		camera.stopStreaming();
	}

	public int detectRings() { //returns 0, 1 or 4
		int pos = -1;

		if(valTop == -1 || valBottom == -1) {
			return -1;
		}

		if (valTop > 100) pos = 4; //top is yellow enough
		else if (valBottom > 100) pos = 1; //top is not yellow but bottom is yellow enough
		else pos = 0;

		telemetry.addData("Values ", valTop + "   " + valBottom);
		telemetry.addData("pos ", pos);
		telemetry.update();

		numOfRings = pos;
		return pos;
	}

	public int getDetectedRings(){
		return numOfRings;
	}

	static class RingPipeline extends OpenCvPipeline {
		enum Stage {//color difference. greyscale
			detection,//includes outlines
			THRESHOLD,//b&w
			RAW_IMAGE,//displays raw view
		}

		private Stage stageToRenderToViewport = Stage.detection;
		private Stage[] stages = Stage.values();

		Mat yCbCrChan2Mat = new Mat(); //the popular JPEG image format
		Mat thresholdMat = new Mat();
		Mat all = new Mat();
		List<MatOfPoint> contoursList = new ArrayList<>();

		@Override
		public void onViewportTapped() {
			/*
			 * Note that this method is invoked from the UI thread
			 * so whatever we do here, we must do quickly.
			 */
			int currentStageNum = stageToRenderToViewport.ordinal();
			int nextStageNum = currentStageNum + 1;

			if (nextStageNum >= stages.length) {
				nextStageNum = 0;
			}
			stageToRenderToViewport = stages[nextStageNum];
		}

		@Override
		public Mat processFrame(Mat input) {
			contoursList.clear();
			/*
			 * This pipeline finds the contours of yellow blobs such as the Gold Mineral
			 * from the Rover Ruckus game.
			 */

			//color diff cb.
			//lower cb = more blue = skystone = white
			//higher cb = less blue = yellow stone = grey
			Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
			Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

			//b&w
			Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

			//outline/contour
			Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
			yCbCrChan2Mat.copyTo(all);//copies mat object
			//Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours

			//get values from frame
			double[] pixBottom = thresholdMat.get((int) (input.rows() * bottomPos[1]), (int) (input.cols() * bottomPos[0]));//gets value at circle
			valBottom = (int) pixBottom[0];

			double[] pixTop = thresholdMat.get((int) (input.rows() * topPos[1]), (int) (input.cols() * topPos[0]));//gets value at circle
			valTop = (int) pixTop[0];

			//create two points
			Point pointBottom = new Point((int) (input.cols() * bottomPos[0]), (int) (input.rows() * bottomPos[1]));
			Point pointTop = new Point((int) (input.cols() * topPos[0]), (int) (input.rows() * topPos[1]));
			//draw circles on those points
			Imgproc.circle(all, pointBottom, 5, RED, 1);//draws circle
			Imgproc.circle(all, pointTop, 5, RED, 1);//draws circle

			//draw 2 rectangles
			Imgproc.rectangle(//top
					all,
					new Point(
							input.cols() * (topPos[0] - rectWidth / 2),
							input.rows() * (topPos[1] - rectHeight)),
					new Point(
							input.cols() * (topPos[0] + rectWidth / 2),
							input.rows() * (topPos[1] + rectHeight)),
					GREEN, 2);
			Imgproc.rectangle(//bottom
					all,
					new Point(
							input.cols() * (bottomPos[0] - rectWidth / 2),
							input.rows() * (bottomPos[1] - rectHeight / 2)),
					new Point(
							input.cols() * (bottomPos[0] + rectWidth / 2),
							input.rows() * (bottomPos[1] + rectHeight / 2)),
					GREEN, 2);

			switch (stageToRenderToViewport) {
				case THRESHOLD:
					return thresholdMat;
				case detection:
					return all;
				case RAW_IMAGE:
					return input;
				default:
					return input;
			}
		}
	}
}
