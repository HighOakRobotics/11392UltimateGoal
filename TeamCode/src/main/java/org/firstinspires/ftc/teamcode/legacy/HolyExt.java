package org.firstinspires.ftc.teamcode.legacy;

//import com.acmerobotics.roadrunner.control.PIDFController;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.legacy.AutoUtil;
import org.firstinspires.ftc.teamcode.legacy.GenUtil;
import org.firstinspires.ftc.teamcode.legacy.MathUtil;
import org.firstinspires.ftc.teamcode.legacy.advancednav.MotionProfile;
import org.firstinspires.ftc.teamcode.legacy.advancednav.SetPoint;
import org.firstinspires.ftc.teamcode.legacy.advancednav.Trajectory;

import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.inchesToEncoderTicks;

//import org.apache.commons.math3.analysis.UnivariateFunction;
//import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
//import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;

//This class provides basic odometry methods and allows a
public class HolyExt extends Holonomic {
	private int currentXPos = 0;
	private int currentYPos = 0;
	public BNO055IMU imu;
	private LinearOpMode justPutThis;
	public Telemetry telemetry;

	private double Kp = 10;
	private double Ki = 0;
	private double Kd = 3;

	private double KpTurn;
	private double KiTurn;
	private double KdTurn;

	private double maxI;

	private int[] lastTickPos = new int[4];

	public HolyExt(HardwareMap hwMap, LinearOpMode justPutThis) {
		super(hwMap);
		this.justPutThis = justPutThis;
	}

	public HolyExt(HardwareMap hwMap, Telemetry telemetry, LinearOpMode justPutThis) {
		super(hwMap/*, teleBug*/);
		this.justPutThis = justPutThis;
		this.telemetry = telemetry;
	}

	public HolyExt(HardwareMap hwMap, Telemetry telemetry, double d, double s, double t, LinearOpMode justPutThis) {
		super(hwMap/*, telebug*/, d, s, t, false);
		this.telemetry = telemetry;
		this.justPutThis = justPutThis;
	}

	public HolyExt(HardwareMap hwMap, Telemetry telemetry, double d, double s, double t, boolean inverted, LinearOpMode justPutThis) {
		super(hwMap, d, s, t, inverted);
		this.telemetry = telemetry;
		this.justPutThis = justPutThis;
	}

	public void init() {
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled = true;
		parameters.loggingTag = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
		// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
		// and named "imu".
		imu = this.hwMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);

		super.init();

		lastTickPos[0] = leftFront.getCurrentPosition();
		lastTickPos[1] = rightFront.getCurrentPosition();
		lastTickPos[2] = leftBack.getCurrentPosition();
		lastTickPos[3] = rightBack.getCurrentPosition();

		KpTurn = rightFront.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).p;
		KiTurn = rightFront.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).i;
		KdTurn = rightFront.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).d;

		maxI = 5;
	}

	public void runWithEncoders() {
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void straightLineToPosition(int x, int y, double power) {
		AutoUtil.turnToAbsolutePosition((int) Math.atan2(y, x), 0.7, this, imu, justPutThis);
		//turnWithGyro((Math.atan2(y, x) - imu.getAngularOrientation().firstAngle), 0.7);
		super.driveByInch(MathUtil.pyth(currentXPos - x, currentYPos - y), power);
		while (justPutThis.opModeIsActive() && !justPutThis.isStopRequested() && super.isInTarget()) {
			//Telemetry code?
		}
		super.stopTargeting();
		currentXPos += x;
		currentYPos += y;
	}

	public void sCurveDrive(double distance, double maxPower) {
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		MotionProfile sCurve = new MotionProfile(distance, maxPower);
		SetPoint setpoint;
		//arrays of length 4, one for each wheel
		double[] error = new double[4];
		double[] lastError = new double[4];
		double[] totalError = new double[4];
		ElapsedTime time = new ElapsedTime();
		int curVelo;
		while (justPutThis.opModeIsActive() && !justPutThis.isStopRequested() && time.seconds() < sCurve.totalStageDuration()) {
			setpoint = sCurve.lookUpSetpoint(time.seconds());
			//error is defined as the diff. between the setpoint's distance and actual distance from the encoder
			//(how far off it is from the projected graph)
			error[0] = setpoint.getDist() - encoderTicksToInches(leftFront.getCurrentPosition());
			error[1] = setpoint.getDist() - encoderTicksToInches(rightFront.getCurrentPosition());
			error[2] = setpoint.getDist() - encoderTicksToInches(leftBack.getCurrentPosition());
			error[3] = setpoint.getDist() - encoderTicksToInches(rightBack.getCurrentPosition());

			//velocity + Kp(error) + Ki(integral) - Kd(last_error - error)
			curVelo = inchesToEncoderTicks(setpoint.getVelo());
			leftFront.setVelocity(curVelo + Kp * error[0] + Ki * totalError[0] - Kd * lastError[0] - error[0]);
			rightFront.setVelocity(curVelo + Kp * error[1] + Ki * totalError[1] - Kd * lastError[1] - error[1]);
			leftBack.setVelocity(curVelo + Kp * error[2] + Ki * totalError[2] - Kd * lastError[2] - error[2]);
			rightBack.setVelocity(curVelo + Kp * error[3] + Ki * totalError[3] - Kd * lastError[3] - error[3]);

			//updates error, adds on to total error
			lastError = error;
			for (int i = 0; i < totalError.length; i++) {
				totalError[i] += error[i];
			}
			try {
				GenUtil.sleep(20, justPutThis);
			} catch (Exception e) {
			}
		}
		super.stopTargeting();
	}

	public int getCurrentXPos() {
		return currentXPos;
	}

	public int getCurrentYPos() {
		return currentYPos;
	}


	public void FOTD(Trajectory trajectory) {
		double currentX = trajectory.getStartX();
		double currentAngle;
		//UnivariateFunction deriv = trajectory.getSpline().derivative();
		while (justPutThis.opModeIsActive() && !justPutThis.isStopRequested() && currentX < trajectory.getEndX()) { //make it detect when its close to the endpoint
			Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
			currentAngle = AngleUnit.RADIANS.normalize(AngleUnit.RADIANS.fromUnit(angles.angleUnit, angles.firstAngle));
			//GOOD CODE

			//Converting from IMU units to joystick units (very poorly)
			currentAngle += Math.PI;
			currentAngle *= -1;

			//Make sure it's in the interval -pi to pi (OPTIMIZED)
			currentAngle %= 2 * Math.PI;
			if (currentAngle > Math.PI) {
				currentAngle -= 2 * Math.PI;
			}

			double speed = 0.5;
			//TODO handle profile management for slowing during turns

			//This is negative because the axes were flipped and this fixed it and we don't know why but it works so we don't care.
			//double angle = Math.atan(deriv.value(currentX));

			//super.aDriveRaw(angle - currentAngle, speed, 0.3);
		}
	}

	public void turnLeftRelativeSynch(double degrees, double power) {
		AutoUtil.turnLeft(degrees, this, imu, justPutThis);
	}

	public void turnRightRelativeSynch(double degrees, double power) {
		AutoUtil.turnRight(degrees, this, imu, justPutThis);
	}

	public void turn2StepToAbsoluteSynch(double degrees, double power1, double margin1, double margin2, double power2) {
		AutoUtil.imu2StepTurnAbsoluteSynch(degrees, power1, power2, margin1, margin2, this, imu, justPutThis, telemetry);
	}

	public void turn2StepToAbsoluteZeroSynch(double power1, double power2, double margin1, double margin2) {
		turn2StepToAbsoluteSynch(0, power1, power2, margin1, margin2);
	}

	public void turn2StepToAbsoluteZeroSynch() {
		turn2StepToAbsoluteZeroSynch(0.5, 0.375, 1, 1);
	}

	public void setPIDTurnCoefficients(PIDFCoefficients pidf) {
		KpTurn = pidf.p;
		KiTurn = pidf.i;
		KdTurn = pidf.d;
	}

	public void setMaxI(double newMaxI) {
		maxI = newMaxI;
	}

	public void turnPIDAbsoluteSynch(double degrees, double power) {
		AutoUtil.imuPIDTurnAbsoluteSynch(degrees, power, imu, this, justPutThis, telemetry, KpTurn, KiTurn, KdTurn, maxI, 1);
	}

	public void turnPIDToAbsoluteZeroSynch(double power) {
		turnPIDAbsoluteSynch(0, power);
	}

	public void turnPIDToAbsoluteZeroSynch() {
		turnPIDToAbsoluteZeroSynch(1.0);
	}

	public void turnWithEncoder(double input) {
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		//
		leftFront.setPower(input);
		rightFront.setPower(-input);
		leftBack.setPower(input);
		rightBack.setPower(-input);
	}

	public void updatePos() {
		//lastTickPos[]
		int leftFrontDiff = leftFront.getCurrentPosition() - lastTickPos[0];
		int rightFrontDiff = rightFront.getCurrentPosition() - lastTickPos[1];
		int leftBackDiff = leftBack.getCurrentPosition() - lastTickPos[2];
		int rightBackDiff = rightBack.getCurrentPosition() - lastTickPos[3];

		currentXPos += encoderTicksToInches((int) ((leftFrontDiff * -1 / Math.sqrt(2)) + (rightFrontDiff * 1 / Math.sqrt(2)) + (leftBackDiff * 1 / Math.sqrt(2)) + (rightBackDiff * -1 / Math.sqrt(2))));
		currentYPos += encoderTicksToInches((int) ((leftFrontDiff * 1 / Math.sqrt(2)) + (rightFrontDiff * 1 / Math.sqrt(2)) + (leftBackDiff * 1 / Math.sqrt(2)) + (rightBackDiff * 1 / Math.sqrt(2))));

		lastTickPos[0] = leftFront.getCurrentPosition();
		lastTickPos[1] = rightFront.getCurrentPosition();
		lastTickPos[2] = leftBack.getCurrentPosition();
		lastTickPos[3] = rightBack.getCurrentPosition();
	}
}
