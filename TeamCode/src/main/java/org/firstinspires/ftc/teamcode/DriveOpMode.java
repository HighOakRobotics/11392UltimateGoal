package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.ftc11392.sequoia.task.RunTask;
import com.ftc11392.sequoia.task.Task;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Shooter11392;
import org.firstinspires.ftc.teamcode.subsystem.Shooter2W;
import org.firstinspires.ftc.teamcode.subsystem.positioning.OdometrySensor;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.task.Shooter11392ControlTask;
import org.firstinspires.ftc.teamcode.task.Shooter2WControlTask;
import org.firstinspires.ftc.teamcode.task.ShooterControlTask;

@TeleOp(name="Tele 11392", group="11392")
public class DriveOpMode extends SequoiaOpMode {
	Shooter11392 shooter = new Shooter11392();
	//OdometrySensor odometry = new OdometrySensor();
	//MecanumSubsystem drivetrain = new MecanumSubsystem(odometry.getPositionSupplier());

	@Override
	public void initTriggers() { }

	@Override
	public void runTriggers() {
		gamepad1H.aToggleButton().risingWithCancel(//new ParallelTaskBundle(
				new Shooter11392ControlTask(shooter)//,
				//new RunTask(() -> {
				//	telemetry.addData("shooter", "running");
				//}, null)
				//)
		);
		//gamepad1H.sticksButton(0.05).onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
	}
}