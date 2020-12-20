package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.CyclicBarrier;

public class Holonomic {
	public DcMotorEx leftFront;
	public DcMotorEx leftBack;
	public DcMotorEx rightFront;
	public DcMotorEx rightBack;

	protected HardwareMap hwMap;
	//protected TeleBug teleBug;
	//private Positron positron;

	//TODO change these values with testing
	private double dMultiplier = 1;
	private double sMultiplier = 1;
	private double tMultiplier = 0.5;
	private int inv;

	/* These fields are used for configuring encoder drive methods.
	 * Values according to NeveRest 40 specs:
	 * Ticks per Motor Rev = 1440 (and that's it.)
	 * According to Brian, average wheel diameter is 4.0 inches for our mecanums.
	 */
	private static double TICKS_PER_MOTOR_REV     = 1120;
	private static double WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
	private static double TICKS_PER_INCH = TICKS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

	private static final int POSITIONING_TOLERANCE = 10; //The amount of ticks the DriveByInch method should be allowed to deviate by

	private static final int TURN_COMPENSATION = 0;
	private static final int DRIVE_COMPENSATION = 0;
	private static final int STRAFE_COMPENSATION = 0;

	public Holonomic (HardwareMap hwMap){
		this.hwMap = hwMap;
	}

	public Holonomic (HardwareMap hwMap, double d, double s, double t){
		this(hwMap, d, s, t, false);
	}

	public Holonomic (HardwareMap hwMap, double d, double s, double t, boolean inverted){
		this(hwMap);
		dMultiplier = d;
		sMultiplier = s;
		tMultiplier = t;
		inv = inverted ? -1 : 1;
	}

	public void init(){
		leftFront = hwMap.get(DcMotorEx.class, "frontLeft");
		rightFront = hwMap.get(DcMotorEx.class, "frontRight");
		leftBack = hwMap.get(DcMotorEx.class, "backLeft");
		rightBack = hwMap.get(DcMotorEx.class, "backRight");

		leftFront.setTargetPositionTolerance(POSITIONING_TOLERANCE);
		rightFront.setTargetPositionTolerance(POSITIONING_TOLERANCE);
		leftBack.setTargetPositionTolerance(POSITIONING_TOLERANCE);
		rightBack.setTargetPositionTolerance(POSITIONING_TOLERANCE);
	}

	public void setPID(double p, double i, double d) {
		leftFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(new PIDCoefficients(p,i,d)));
		rightFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(new PIDCoefficients(p,i,d)));
		leftBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(new PIDCoefficients(p,i,d)));
		rightBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(new PIDCoefficients(p,i,d)));
	}

	public void setBrake() {
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void stop(){
		leftFront.setPower(0);
		rightBack.setPower(0);
		leftBack.setPower(0);
		rightFront.setPower(0);
	}

	public void stopTargeting() {
		stop();
		leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	public void dstDrive(double drive, double strafe, double turn){
        /*
        //Driving and strafing were switched, so I switched driveMult and strafeMult and inverted drive.
        //double strafeMult = -drive * dMultiplier;
        //double driveMult = strafe * sMultiplier;
*/
		drive = drive * dMultiplier;
		strafe = strafe * sMultiplier;
		turn = turn * tMultiplier;
/*
        double angle = Math.atan2(driveMult, strafeMult);
        double power = Math.sqrt(Math.pow(driveMult, 2) + Math.pow(strafeMult, 2));
        aDriveRaw(angle, power, turnMult);*/
		double frontLeft = Range.clip(drive - strafe - inv*turn,-1,1);
		double frontRight = Range.clip(-drive - strafe - inv*turn,-1,1);
		double backLeft = Range.clip(drive + strafe - inv*turn,-1,1);
		double backRight = Range.clip(-drive + strafe - inv*turn,-1,1);

		rightFront.setPower(inv*frontRight);
		leftBack.setPower(inv*backLeft);
		rightBack.setPower(inv*backRight);
		leftFront.setPower(inv*frontLeft);
	}

	//uses degrees
	public void aDrive(double angle, double power, double turn){
		double turnMult = turn * tMultiplier;
		double angleRad = angle * (Math.PI / 180);
		aDriveRaw(angleRad, power, turnMult);
	}

	//ROBOT ORIENTED
	//uses radians
	public void aDriveRaw(double angle, double power, double turn){
		double frontLeft = (-power * Math.cos((Math.PI / 4.0) - angle)) + (turn*inv);
		double frontRight = (power * Math.cos((Math.PI / 4.0) + angle)) + (turn*inv);
		double backLeft = (-power * Math.cos((Math.PI / 4.0) + angle)) + (turn*inv);
		double backRight = (power * Math.cos((Math.PI / 4.0) - angle)) + (turn*inv);

		//safe drive
		if (Math.abs(frontLeft) < 0.05 ) frontLeft = 0.0;
		if (Math.abs(frontRight) < 0.05) frontRight = 0.0;
		if (Math.abs(backLeft) < 0.05 ) backLeft = 0.0;
		if (Math.abs(backRight) < 0.05) backRight = 0.0;

		rightFront.setPower(inv*frontRight);
		leftBack.setPower(inv*backLeft);
		rightBack.setPower(inv*backRight);
		leftFront.setPower(inv*frontLeft);
	}

	public void initRT(double power) {
		leftBack.setTargetPosition(leftBack.getCurrentPosition());
		leftFront.setTargetPosition(leftFront.getCurrentPosition());
		rightBack.setTargetPosition(rightBack.getCurrentPosition());
		rightFront.setTargetPosition(rightFront.getCurrentPosition());
		setRT(power);
	}

	public void setRT(double power) {
		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		rightFront.setPower(power);
		leftBack.setPower(power);
		rightBack.setPower(power);
		leftFront.setPower(power);
	}

	public void driveByInchSynch (double inches, double power, LinearOpMode justPutThis){
		int targetPos = (int)(inches * TICKS_PER_INCH);

		leftBack.setTargetPosition((leftBack.getCurrentPosition() - targetPos)*inv);
		leftFront.setTargetPosition((leftFront.getCurrentPosition() - targetPos)*inv);
		rightBack.setTargetPosition((rightBack.getCurrentPosition() + targetPos)*inv);
		rightFront.setTargetPosition((rightFront.getCurrentPosition() + targetPos)*inv);

		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		leftFront.setPower(inv*power);
		leftBack.setPower(inv*power);
		rightFront.setPower(inv*power);
		rightBack.setPower(inv*power);


		while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
			if (justPutThis.isStopRequested() || !justPutThis.opModeIsActive()) {
				this.stop();
				return;
			}
		}

		justPutThis.sleep(150);
		//this.stop();
	}
	public void driveByInchSynchRT(double inches, double power, LinearOpMode justPutThis){
		int targetPos = (int)(inv * inches * TICKS_PER_INCH);

		leftBack.setTargetPosition((leftBack.getTargetPosition() - targetPos)*inv);
		leftFront.setTargetPosition((leftFront.getTargetPosition() - targetPos)*inv);
		rightBack.setTargetPosition((rightBack.getTargetPosition() + targetPos)*inv);
		rightFront.setTargetPosition((rightFront.getTargetPosition() + targetPos)*inv);

		while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
			if (justPutThis.isStopRequested() || !justPutThis.opModeIsActive()) {
				this.stop();
				return;
			}
		}
		justPutThis.sleep(100);
	}

	@Deprecated
	public void driveByInch(double inches, double power){
		int targetPos = (int)(inches * TICKS_PER_INCH);

		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		rightFront.setPower(-power);
		leftBack.setPower(-power);
		rightBack.setPower(-power);
		leftFront.setPower(-power);

		rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPos);
		leftBack.setTargetPosition(leftBack.getCurrentPosition() - targetPos);
		rightBack.setTargetPosition(rightBack.getCurrentPosition() + targetPos);
		leftFront.setTargetPosition(leftFront.getCurrentPosition() - targetPos);

		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void driveByInchWeird(double inches, final double power){
		final int targetPos = (int)(inches * TICKS_PER_INCH);

		final CyclicBarrier gate = new CyclicBarrier(5);

		Thread t1 = new Thread(){
			public void run(){
				try {
					gate.await();
				} catch (Exception e){}
				//do stuff
				rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				rightFront.setPower(-power);
				rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPos);
				rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			}};
		Thread t2 = new Thread(){
			public void run(){
				try {
					gate.await();
				} catch (Exception e){}
				//do stuff
				leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				leftFront.setPower(-power);
				leftFront.setTargetPosition(rightFront.getCurrentPosition() + targetPos);
				leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			}};
		Thread t3 = new Thread(){
			public void run(){
				try {
					gate.await();
				} catch (Exception e){}
				//do stuff
				rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				rightBack.setPower(-power);
				rightBack.setTargetPosition(rightFront.getCurrentPosition() + targetPos);
				rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			}};
		Thread t4 = new Thread(){
			public void run(){
				try {
					gate.await();
				} catch (Exception e){}
				//do stuff
				leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				leftBack.setPower(-power);
				leftBack.setTargetPosition(rightFront.getCurrentPosition() + targetPos);
				leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			}};

		t1.start();
		t2.start();
		t3.start();
		t4.start();

		try {
			gate.await();
		} catch (Exception e){}
	}
	//positive is right, negative is left for inches
	public void ourStrafeByInch (double inches, double speed){
		//
		final int targetPos = (int)(inches * TICKS_PER_INCH);
		//
		leftBack.setTargetPosition((leftBack.getCurrentPosition() + targetPos + STRAFE_COMPENSATION + TURN_COMPENSATION - DRIVE_COMPENSATION)*inv);
		leftFront.setTargetPosition((leftFront.getCurrentPosition() - targetPos - STRAFE_COMPENSATION + TURN_COMPENSATION - DRIVE_COMPENSATION)*inv);
		rightBack.setTargetPosition((rightBack.getCurrentPosition() + targetPos + STRAFE_COMPENSATION + TURN_COMPENSATION + DRIVE_COMPENSATION)*inv);
		rightFront.setTargetPosition((rightFront.getCurrentPosition() - targetPos - STRAFE_COMPENSATION + TURN_COMPENSATION + DRIVE_COMPENSATION)*inv);
		//
		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		//
		leftFront.setPower(-speed*inv);
		leftBack.setPower(-speed*inv);
		rightFront.setPower(-speed*inv);
		rightBack.setPower(-speed*inv);
		//
		while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()){}
		//rightFront.setPower(0);
		//leftFront.setPower(0);
		//rightBack.setPower(0);
		//leftBack.setPower(0);
		return;
	}

	public void ourStrafeByInch (double inches, double speed, LinearOpMode justPutThis){
		ourStrafeByInch(inches, speed);
		justPutThis.sleep(150);
	}

	public void ourStrafeByInchRT(double inches, double speed, LinearOpMode justPutThis){
		//

		final int targetPos = (int)(inv * inches * TICKS_PER_INCH);
		//
		leftBack.setTargetPosition((leftBack.getTargetPosition() + targetPos + STRAFE_COMPENSATION + TURN_COMPENSATION - DRIVE_COMPENSATION)*inv);
		leftFront.setTargetPosition((leftFront.getTargetPosition() - targetPos - STRAFE_COMPENSATION + TURN_COMPENSATION - DRIVE_COMPENSATION)*inv);
		rightBack.setTargetPosition((rightBack.getTargetPosition() + targetPos + STRAFE_COMPENSATION + TURN_COMPENSATION + DRIVE_COMPENSATION)*inv);
		rightFront.setTargetPosition((rightFront.getTargetPosition() - targetPos - STRAFE_COMPENSATION + TURN_COMPENSATION + DRIVE_COMPENSATION)*inv);
		//
		while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()){}
		justPutThis.sleep(100);
		return;
	}

	//Assumes dst multipliers are set correctly
	public void strafeRightByInch(double inches, double power){
		int targetPos = (int)(inches * TICKS_PER_INCH);

		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		rightFront.setPower(-power*inv);
		leftBack.setPower(-power*inv);
		rightBack.setPower(-power*inv);
		leftFront.setPower(-power*inv);

		rightFront.setTargetPosition((rightFront.getCurrentPosition() - targetPos)*inv);
		leftBack.setTargetPosition((leftBack.getCurrentPosition() + targetPos)*inv);
		rightBack.setTargetPosition((rightBack.getCurrentPosition() + targetPos)*inv);
		leftFront.setTargetPosition((leftFront.getCurrentPosition() - targetPos)*inv);

		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}
	//These are twin methods, only thing different is the direction of the wheels.
	public void strafeLeftByInch(double inches, double power){
		int targetPos = (int)(inches * TICKS_PER_INCH);

		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		rightFront.setPower(-power*inv);
		leftBack.setPower(-power*inv);
		rightBack.setPower(-power*inv);
		leftFront.setPower(-power*inv);

		rightFront.setTargetPosition((rightFront.getCurrentPosition() + targetPos)*inv);
		leftBack.setTargetPosition((leftBack.getCurrentPosition() - targetPos)*inv);
		rightBack.setTargetPosition((rightBack.getCurrentPosition() - targetPos)*inv);
		leftFront.setTargetPosition((leftFront.getCurrentPosition() + targetPos)*inv);

		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void dstEncodedSynch(double forwardInches, double strafeInches, double turnDegrees, double maxPower, LinearOpMode justPutThis){
        /*
         - Find the computed target value with the largest magnitude in ticks.
         - Calculate the ratio between the D, S, and T component tick values and the combined value.
         - Multiply the ratios with `maxPower` and store them as temporary power constants.
         - Set each motor power to the associated sum of the product of the temporary power constants and the associated component D, S, and T values of each motor.
         */
		double targetInches = Math.sqrt(Math.pow(forwardInches, 2) + Math.pow(strafeInches, 2));
		int targetPos = (int)(targetInches * TICKS_PER_INCH);

		leftBack.setTargetPosition((leftBack.getCurrentPosition() - targetPos)*inv);
		leftFront.setTargetPosition((leftFront.getCurrentPosition() - targetPos)*inv);
		rightBack.setTargetPosition((rightBack.getCurrentPosition() + targetPos)*inv);
		rightFront.setTargetPosition((rightFront.getCurrentPosition() + targetPos)*inv);

		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		leftFront.setPower(inv*maxPower);
		leftBack.setPower(inv*maxPower);
		rightFront.setPower(inv*maxPower);
		rightBack.setPower(inv*maxPower);


		while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
			if (justPutThis.isStopRequested() || !justPutThis.opModeIsActive()) {
				this.stop();
				return;
			}
		}

		this.stop();
	}

	public void turnOnBrakes() {
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void resetEncoder() {
		leftFront .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftBack  .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightBack .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}
	public void turnOffBrakes() {
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	public void setTPR(double tpr) {
		TICKS_PER_MOTOR_REV = tpr;
		TICKS_PER_INCH = TICKS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
	}

	public boolean isInTarget() {
		return  Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) <= POSITIONING_TOLERANCE &&
				Math.abs(rightFront.getCurrentPosition() - rightFront.getTargetPosition()) <= POSITIONING_TOLERANCE &&
				Math.abs(leftBack.getCurrentPosition() - leftBack.getTargetPosition()) <= POSITIONING_TOLERANCE &&
				Math.abs(rightBack.getCurrentPosition() - rightBack.getTargetPosition()) <= POSITIONING_TOLERANCE;
	}
}
