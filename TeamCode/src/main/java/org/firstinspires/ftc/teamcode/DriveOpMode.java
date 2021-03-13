package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Loader;
import org.firstinspires.ftc.teamcode.subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.subsystem.Shimmier;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;
import org.firstinspires.ftc.teamcode.subsystem.WobbleGripper;
import org.firstinspires.ftc.teamcode.subsystem.positioning.Position;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.task.LiftControlTask;
import org.firstinspires.ftc.teamcode.task.LoaderPushTask;
import org.firstinspires.ftc.teamcode.task.ResetTiltTask;
import org.firstinspires.ftc.teamcode.task.ShimmyTask;
import org.firstinspires.ftc.teamcode.task.StartShooterTask;
import org.firstinspires.ftc.teamcode.task.StopShooterTask;
import org.firstinspires.ftc.teamcode.task.TiltModeSelectTask;
import org.firstinspires.ftc.teamcode.task.WobbleGripperControlTask;

import java.util.function.Supplier;

@TeleOp(name = "DriveOpMode 11392", group = "11392")
public class DriveOpMode extends SequoiaOpMode {
	Shimmier shimmier = new Shimmier();
	Shooter shooter = new Shooter();
	Intake intake = new Intake();
	Loader loader = new Loader();
	Tilt tilt = new Tilt();
	Lift lift = new Lift();
	WobbleGripper gripper = new WobbleGripper();
	Mecanum drivetrain = new Mecanum();
	Supplier<Position> positionSupplier = drivetrain.mecanum().getPositionSupplier();

	@Override
	public void initTriggers() {

	}

	@Override
	public void runTriggers() {
		drivetrain.mecanum().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		gamepad1H.rightTriggerButton(0.05).whilePressed(new LiftControlTask(50, lift));
		gamepad1H.leftTriggerButton(0.05).whilePressed(new LiftControlTask(-50, lift));
		gamepad1H.aButton()
				.onPress(new ParallelTaskBundle(
						new InstantTask(() -> {intake.setIntakePower(0);}),
						new StartShooterTask(shooter),
						new TiltModeSelectTask(TiltModeSelectTask.Position.SHOOT, tilt)
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
		gamepad1H.rightBumperButton()
				.onRelease(new LoaderPushTask(loader));
		gamepad1H.leftButton()
				.onRelease(new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.CLOSE, gripper));
		gamepad1H.rightButton()
				.onRelease(new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.OPEN, gripper));
		gamepad1H.leftBumperButton()
				.onPress(new ResetTiltTask(tilt));
		gamepad2H.aButton()
				.onPress(new ShimmyTask(shimmier));
	}
}