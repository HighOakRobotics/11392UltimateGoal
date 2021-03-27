package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Lights;
import org.firstinspires.ftc.teamcode.subsystem.Loader;
import org.firstinspires.ftc.teamcode.subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;
import org.firstinspires.ftc.teamcode.subsystem.WobbleGripper;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.LiftControlTask;
import org.firstinspires.ftc.teamcode.task.LoaderPushTask;
import org.firstinspires.ftc.teamcode.task.ShooterControlTask;
import org.firstinspires.ftc.teamcode.task.TiltModeSelectTask;
import org.firstinspires.ftc.teamcode.task.WobbleGripperControlTask;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "DriveOpMode 11392", group = "11392")
public class DriveOpMode extends SequoiaOpMode {
	Shooter shooter = new Shooter();
	Intake intake = new Intake();
	Loader loader = new Loader();
	Lights lights = new Lights();
	Tilt tilt = new Tilt();
	Lift lift = new Lift();
	WobbleGripper gripper = new WobbleGripper();
	Mecanum drivetrain = new Mecanum();
	//OdometrySensor odometry = new OdometrySensor();

	@Override
	public void initTriggers() {

	}

	@Override
	public void runTriggers() {
		scheduler.schedule(new SequentialTaskBundle(
				new WaitTask(80, TimeUnit.SECONDS),
				new InstantTask(() -> {lights.endGame();}),
				new WaitTask(30,TimeUnit.SECONDS),
				new InstantTask( () -> {lights.finalTen();})
		));
		drivetrain.mecanum().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		gamepad1H.rightTriggerButton(0.05).whilePressed(new LiftControlTask(50, lift));
		gamepad1H.leftTriggerButton(0.05).whilePressed(new LiftControlTask(-50, lift));
		gamepad1H.aToggleButton()
				.risingWithCancel(new ParallelTaskBundle(
						new ShooterControlTask(shooter),
						new TiltModeSelectTask(TiltModeSelectTask.Position.SHOOT, tilt)
				));
		gamepad1H.yButton().onPressWithCancel(new InstantTask(() -> {
			intake.setIntakePower(1.0);
		}));
		gamepad1H.xToggleButton()
				.risingWithCancel(new ParallelTaskBundle(
						new IntakeTask(intake),
						new TiltModeSelectTask(TiltModeSelectTask.Position.LOAD, tilt)
				));
		gamepad1H.sticksButton(0.05)
				.onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
		gamepad1H.rightBumperButton()
				.onRelease(new LoaderPushTask(loader).withTimeout(300, TimeUnit.MILLISECONDS));
		//gamepad1H.leftBumperButton()
		//		.onPress(new TiltModeSelectTask(TiltModeSelectTask.Position.POWERSHOT, tilt));
		gamepad1H.leftButton()
				.onRelease(new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.CLOSE, gripper));
		gamepad1H.rightButton()
				.onRelease(new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.OPEN, gripper));
	}
}