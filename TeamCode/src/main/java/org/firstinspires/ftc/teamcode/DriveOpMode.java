package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.Shooter11392ControlTask;

@TeleOp(name="Tele 11392", group="11392")
public class DriveOpMode extends SequoiaOpMode {
	Shooter shooter = new Shooter();
	Intake intake = new Intake();
	//OdometrySensor odometry = new OdometrySensor();
	MecanumSubsystem drivetrain = new MecanumSubsystem();

	@Override
	public void initTriggers() { }

	@Override
	public void runTriggers() {
		gamepad1H.aToggleButton().risingWithCancel(new Shooter11392ControlTask(shooter));
		gamepad1H.xToggleButton().risingWithCancel(new IntakeTask(intake));
		gamepad1H.sticksButton(0.05).onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
	}
}