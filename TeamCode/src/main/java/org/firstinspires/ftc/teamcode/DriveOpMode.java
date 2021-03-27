package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Loader;
import org.firstinspires.ftc.teamcode.subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;
import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;
import org.firstinspires.ftc.teamcode.subsystem.WobbleGripper;
import org.firstinspires.ftc.teamcode.subsystem.positioning.FusionSensor;
import org.firstinspires.ftc.teamcode.subsystem.positioning.OdometrySensor;
import org.firstinspires.ftc.teamcode.subsystem.positioning.Position;
import org.firstinspires.ftc.teamcode.subsystem.positioning.PositionLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.positioning.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.positioning.VSLAMSensor;
import org.firstinspires.ftc.teamcode.task.PointControlTask;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.task.LiftControlTask;
import org.firstinspires.ftc.teamcode.task.LoaderPushTask;
import org.firstinspires.ftc.teamcode.task.ResetTiltTask;
import org.firstinspires.ftc.teamcode.task.ShakeTask;
import org.firstinspires.ftc.teamcode.task.StartShooterTask;
import org.firstinspires.ftc.teamcode.task.StopShooterTask;
import org.firstinspires.ftc.teamcode.task.TiltModeSelectTask;
import org.firstinspires.ftc.teamcode.task.WobbleGripperControlTask;

@TeleOp(name = "DriveOpMode 11392", group = "11392")
public class DriveOpMode extends SequoiaOpMode {
	Shooter shooter = new Shooter();
	Intake intake = new Intake();
	Loader loader = new Loader();
	Tilt tilt = new Tilt();
	WobbleArm wobbleArm = new WobbleArm();
	WobbleGripper gripper = new WobbleGripper();
	Mecanum drivetrain = new Mecanum();
	VSLAMSensor vslam = new VSLAMSensor();
	OdometrySensor odo = new OdometrySensor(
			() -> new TwoWheelLocalizer(hardwareMap,
					() -> drivetrain.mecanum().getRawExternalHeading(),
					() -> drivetrain.mecanum().getRawHeadingVelocity()
			)
	);
	FusionSensor fuse = new FusionSensor(vslam, odo);
	Position initPos = new Position();

	double pointingOutput = 0;
	//Supplier<Position> positionSupplier = drivetrain.mecanum().getPositionSupplier();

	@Override
	public void initTriggers() {
		/*
		try {
			telemetry.log().add("got x", Double.parseDouble(scheduler.getPersistentData("x")));
			telemetry.log().add("got y", Double.parseDouble(scheduler.getPersistentData("y")));
			telemetry.log().add("got rot", Double.parseDouble(scheduler.getPersistentData("rot")));
			initPos = new Position(
							Double.parseDouble(scheduler.getPersistentData("x")),
							Double.parseDouble(scheduler.getPersistentData("y")),
							Double.parseDouble(scheduler.getPersistentData("rot")));
			drivetrain.mecanum().setPoseEstimate(new Pose2d(initPos.getX(), initPos.getY(), initPos.getHeading()));
		} catch (IllegalArgumentException e) {
			initPos = new Position(0.0,0.0,0.0);
			telemetry.log().add("was unable to fetch persistent pose... using 0,0,0");
		}
		 */
		drivetrain.mecanum().setLocalizer(
				new PositionLocalizer(fuse.getPositionSupplier(), fuse.getPositionReset())
		);
	}

	@Override
	public void runTriggers() {
		drivetrain.mecanum().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		gamepad1H.rightTriggerButton(0.05).whilePressed(new LiftControlTask(50, wobbleArm));
		gamepad1H.leftTriggerButton(0.05).whilePressed(new LiftControlTask(-50, wobbleArm));
		gamepad1H.aButton()
				.onPress(new ParallelTaskBundle(
						new InstantTask(() -> {intake.setIntakePower(0);}),
						new StartShooterTask(shooter),
						new ShakeTask(tilt, intake)
				));
		gamepad1H.yButton()
				.onPressWithCancel(new InstantTask(() -> {
			intake.setIntakePower(1.0);
		}));
		gamepad1H.xButton()
				.onPress(new ParallelTaskBundle(
						new InstantTask(() -> {intake.setIntakePower(-0.7);}),
						new StopShooterTask(shooter),
						new TiltModeSelectTask(TiltModeSelectTask.Position.LOAD, tilt)
				));
		gamepad1H.bButton()
				.onPress(new ParallelTaskBundle(
						new InstantTask(() -> {intake.setIntakePower(0);}),
						new StopShooterTask(shooter)
				));
		gamepad1H.sticksButton(0.05)
				.onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
		gamepad2H.sticksButton(0.05)
				.onPressWithCancel(
						new ParallelTaskBundle(
								new PointControlTask(drivetrain, (double d) -> {pointingOutput = d;}),
								new StartEndTask(() -> {drivetrain.mecanum().setDriveABS(
										() -> gamepad2.left_stick_x,
										() -> gamepad2.left_stick_y,
										() -> pointingOutput
								);},
										() -> {drivetrain.mecanum().idle();})
						));
		gamepad1H.rightBumperButton()
				.onRelease(new LoaderPushTask(loader));
		gamepad1H.leftButton()
				.onRelease(new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.CLOSE, gripper));
		gamepad1H.rightButton()
				.onRelease(new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.OPEN, gripper));
		gamepad1H.leftBumperButton()
				.onPress(new ResetTiltTask(tilt));
	}
}