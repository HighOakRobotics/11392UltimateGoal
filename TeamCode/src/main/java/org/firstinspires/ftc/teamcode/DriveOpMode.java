package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Loader;
import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.LoaderPushTask;
import org.firstinspires.ftc.teamcode.task.Shooter11392ControlTask;
import org.firstinspires.ftc.teamcode.task.TransferTask;

@TeleOp(name="Tele 11392", group="11392")
public class DriveOpMode extends SequoiaOpMode {
	Shooter shooter = new Shooter();
	Intake intake = new Intake();
	Transfer transfer = new Transfer();
	Loader loader = new Loader();
	//OdometrySensor odometry = new OdometrySensor();
	MecanumSubsystem drivetrain = new MecanumSubsystem();

	@Override
	public void initTriggers() { }

	@Override
	public void runTriggers() {
		gamepad1H.aToggleButton().risingWithCancel(new Shooter11392ControlTask(shooter));
		gamepad1H.xToggleButton().risingWithCancel(new ParallelTaskBundle(new IntakeTask(intake), new TransferTask(transfer)));
		gamepad1H.sticksButton(0.05).onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
		gamepad1H.bButton().onRelease(new LoaderPushTask(loader));
	}
}