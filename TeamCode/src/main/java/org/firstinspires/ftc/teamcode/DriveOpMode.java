package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;

import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;

public class DriveOpMode extends SequoiaOpMode {
	MecanumSubsystem drivetrain = new MecanumSubsystem();

	@Override
	public void initTriggers() {
	}

	@Override
	public void runTriggers() {
		gamepad1H.sticksButton(0.05).onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
	}
}