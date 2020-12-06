package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.kV;

public class DriveTrainMecanum extends MecanumDrive {
	public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
	public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

	public static double LATERAL_MULTIPLIER = 1;
	private final NanoClock clock;
	private final PIDFController turnController;
	private final DriveConstraints constraints;
	private final TrajectoryFollower follower;
	private final List<Pose2d> poseHistory;
	private final DcMotorEx leftFront;
	private final DcMotorEx leftRear;
	private final DcMotorEx rightRear;
	private final DcMotorEx rightFront;
	private final List<DcMotorEx> motors;
	private final BNO055IMU imu;
	private DoubleSupplier drivePower;
	private DoubleSupplier strafePower;
	private DoubleSupplier turnPower;
	private double currentAngle;
	private double angleRad;
	private double driveAngleOffset;
	private double speed;
	private double angle;
	private double turn;
	private DoubleSupplier xPower;
	private DoubleSupplier yPower;
	private DoubleSupplier headingPower;

	private Mode mode;
	private MotionProfile turnProfile;
	private double turnStart;
	private Pose2d lastPoseOnTurn;

	public DriveTrainMecanum(HardwareMap hardwareMap) {
		super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

		clock = NanoClock.system();

		mode = Mode.IDLE;

		turnController = new PIDFController(HEADING_PID);
		turnController.setInputBounds(0, 2 * Math.PI);

		constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
		follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
				new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

		poseHistory = new ArrayList<>();

		for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
			module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}

		// TODO: adjust the names of the following hardware devices to match your configuration
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);

		driveAngleOffset = 0.0;

		// TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
		// upward (normal to the floor) using a command like the following:
		// BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

		leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
		leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
		rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
		rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

		motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

		for (DcMotorEx motor : motors) {
			MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
			motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
			motor.setMotorType(motorConfigurationType);
		}

		if (RUN_USING_ENCODER) {
			setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}

		setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
			setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
		}

		// TODO: reverse any motors using DcMotor.setDirection()

		// TODO: if desired, use setLocalizer() to change the localization method
		// for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
	}

	public void recalibrateAngleOffset(){
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
		currentAngle = AngleUnit.RADIANS.normalize(AngleUnit.RADIANS.fromUnit(angles.angleUnit,angles.firstAngle));
		driveAngleOffset = currentAngle;
	}

	public void setDriveDST() {
		mode = Mode.DRIVE_DST;
	}

	public void setDriveDST(DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn) {
		drivePower = drive;
		strafePower = strafe;
		turnPower = turn;
		mode = Mode.DRIVE_DST;
	}

	private void setMotorsDST() {
		double d = drivePower.getAsDouble();
		double s = strafePower.getAsDouble();
		double t = turnPower.getAsDouble();

		double v = -d + s + t;
		double v1 = -d - s + t;
		double v2 = d - s + t;
		double v3 = d + s + t;

		setMotorPowers(v, v1, v2, v3);
	}

	public void setDriveABS() {
		mode = Mode.DRIVE_ABS;
	}

	public void setDriveABS(DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn) {
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
		currentAngle = AngleUnit.RADIANS.normalize(AngleUnit.RADIANS.fromUnit(angles.angleUnit,angles.firstAngle));
		currentAngle += Math.PI;
		currentAngle *= -1;
		currentAngle %= 2*Math.PI;
		if(currentAngle > Math.PI){
			currentAngle -= 2*Math.PI;
		}

		double x = Range.clip((strafe.getAsDouble() >= 0 ? 1 : -1) * Math.pow(strafe.getAsDouble(), 2), -1, 1);
		double y = Range.clip((drive.getAsDouble() >= 0 ? 1 : -1) * Math.pow(drive.getAsDouble(), 2), -1, 1);
		speed = Range.clip(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)),-1,1);
		angle = Math.atan2(-x, y);

		angle = angle - (currentAngle - driveAngleOffset);

		this.turn = turn.getAsDouble();

		angleRad = angle * (Math.PI / 180);
		mode = Mode.DRIVE_ABS;
	}

	private void setMotorsABS() {
		double frontLeft = (-speed * Math.cos((Math.PI / 4.0) - angle)) + (turn);
		double frontRight = (speed * Math.cos((Math.PI / 4.0) + angle)) + (turn);
		double backLeft = (-speed * Math.cos((Math.PI / 4.0) + angle)) + (turn);
		double backRight = (speed * Math.cos((Math.PI / 4.0) - angle)) + (turn);

		//safe drive
		if (Math.abs(frontLeft) < 0.05 ) frontLeft = 0.0;
		if (Math.abs(frontRight) < 0.05) frontRight = 0.0;
		if (Math.abs(backLeft) < 0.05 ) backLeft = 0.0;
		if (Math.abs(backRight) < 0.05) backRight = 0.0;

		setMotorPowers(frontLeft, backLeft, backRight, frontRight);
	}

	public void idle() {
		mode = Mode.IDLE;
	}

	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
		return new TrajectoryBuilder(startPose, constraints);
	}

	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
		return new TrajectoryBuilder(startPose, reversed, constraints);
	}

	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
		return new TrajectoryBuilder(startPose, startHeading, constraints);
	}

	public void turn(double angle) {
		double heading = getPoseEstimate().getHeading();

		lastPoseOnTurn = getPoseEstimate();

		turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
				new MotionState(heading, 0, 0, 0),
				new MotionState(heading + angle, 0, 0, 0),
				constraints.maxAngVel,
				constraints.maxAngAccel,
				constraints.maxAngJerk
		);

		turnStart = clock.seconds();
		mode = Mode.TURN;
	}

	public void followTrajectory(Trajectory trajectory) {
		follower.followTrajectory(trajectory);
		mode = Mode.FOLLOW_TRAJECTORY;
	}

	public Pose2d getLastError() {
		switch (mode) {
			case FOLLOW_TRAJECTORY:
				return follower.getLastError();
			case TURN:
				return new Pose2d(0, 0, turnController.getLastError());
			case IDLE:
			case DRIVE_DST:
			case DRIVE_ABS:
				return new Pose2d();
		}
		throw new AssertionError();
	}

	public void update() {
		updatePoseEstimate();

		Pose2d currentPose = getPoseEstimate();
		Pose2d lastError = getLastError();

		poseHistory.add(currentPose);

		switch (mode) {
			case IDLE:
				setMotorPowers(0, 0, 0, 0);
				break;
			case DRIVE_DST:
				setMotorsDST();
				break;
			case DRIVE_ABS:
				setMotorsABS();
			case TURN: {
				double t = clock.seconds() - turnStart;

				MotionState targetState = turnProfile.get(t);

				turnController.setTargetPosition(targetState.getX());

				double correction = turnController.update(currentPose.getHeading());

				double targetOmega = targetState.getV();
				double targetAlpha = targetState.getA();
				setDriveSignal(new DriveSignal(new Pose2d(
						0, 0, targetOmega + correction
				), new Pose2d(
						0, 0, targetAlpha
				)));

				Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

				if (t >= turnProfile.duration()) {
					mode = Mode.IDLE;
					setDriveSignal(new DriveSignal());
				}

				break;
			}
			case FOLLOW_TRAJECTORY: {
				setDriveSignal(follower.update(currentPose));

				Trajectory trajectory = follower.getTrajectory();

				if (!follower.isFollowing()) {
					mode = Mode.IDLE;
					setDriveSignal(new DriveSignal());
				}

				break;
			}
		}
	}

	public boolean isBusy() {
		return mode != Mode.IDLE;
	}

	public void setMode(DcMotor.RunMode runMode) {
		for (DcMotorEx motor : motors) {
			motor.setMode(runMode);
		}
	}

	public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
		for (DcMotorEx motor : motors) {
			motor.setZeroPowerBehavior(zeroPowerBehavior);
		}
	}

	public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
		PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
		return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
	}

	public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
		for (DcMotorEx motor : motors) {
			motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
					coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
			));
		}
	}

	@Override
	public List<Double> getWheelPositions() {
		List<Double> wheelPositions = new ArrayList<>();
		for (DcMotorEx motor : motors) {
			wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
		}
		return wheelPositions;
	}

	public List<Double> getWheelVelocities() {
		List<Double> wheelVelocities = new ArrayList<>();
		for (DcMotorEx motor : motors) {
			wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
		}
		return wheelVelocities;
	}

	@Override
	public void setMotorPowers(double v, double v1, double v2, double v3) {
		leftFront.setPower(v);
		leftRear.setPower(v1);
		rightRear.setPower(v2);
		rightFront.setPower(v3);
	}

	@Override
	public double getRawExternalHeading() {
		return imu.getAngularOrientation().firstAngle;
	}

	public enum Mode {
		DRIVE_DST,
		DRIVE_ABS, // TODO implement absolute driving from sensor fusion heading
		IDLE,
		TURN,
		FOLLOW_TRAJECTORY
	}
}
