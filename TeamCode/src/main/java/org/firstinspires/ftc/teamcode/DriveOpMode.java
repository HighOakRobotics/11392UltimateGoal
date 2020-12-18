package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Loader;
import org.firstinspires.ftc.teamcode.subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.LoaderPushTask;
import org.firstinspires.ftc.teamcode.task.ShooterControlTask;
import org.firstinspires.ftc.teamcode.task.TiltModeSelectTask;

@TeleOp(name = "Tele 11392", group = "11392")
public class DriveOpMode extends SequoiaOpMode {
	Shooter shooter = new Shooter();
	Intake intake = new Intake();
	Loader loader = new Loader();
	Tilt tilt = new Tilt();
	Lift lift = new Lift();
	Mecanum drivetrain = new Mecanum();
	//OdometrySensor odometry = new OdometrySensor();

	@Override
	public void initTriggers() {
		drivetrain.mecanum().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	public void runTriggers() {
		gamepad1H.aToggleButton()
				.risingWithCancel(new ShooterControlTask(shooter));
		gamepad1H.xToggleButton()
				.risingWithCancel(new IntakeTask(intake));
		gamepad1H.sticksButton(0.05)
				.onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
		gamepad1H.bButton()
				.onRelease(new LoaderPushTask(loader));
		gamepad1H.rightButton()
				.onRelease(new TiltModeSelectTask(TiltModeSelectTask.Positions.SHOOT,tilt));
		gamepad1H.leftButton()
				.onRelease(new TiltModeSelectTask(TiltModeSelectTask.Positions.LOAD, tilt));
	}
}