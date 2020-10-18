package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Shooter2W;
import org.firstinspires.ftc.teamcode.task.GamepadDriveTask;
import org.firstinspires.ftc.teamcode.task.Shooter2WControlTask;

@TeleOp
public class DriveOpMode extends SequoiaOpMode {
	//MecanumSubsystem drivetrain = new MecanumSubsystem();
	Shooter2W shooter = new Shooter2W();

	@Override
	public void initTriggers() { }

	@Override
	public void runTriggers() {
		gamepad1H.downButton().onPressWithCancel(new Shooter2WControlTask(shooter));
		//gamepad1H.sticksButton(0.05).onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
	}
}