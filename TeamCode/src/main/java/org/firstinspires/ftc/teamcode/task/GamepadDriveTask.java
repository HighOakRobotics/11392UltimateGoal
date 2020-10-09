package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.StartEndTask;
import com.ftc11392.sequoia.task.Task;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;

public class GamepadDriveTask extends StartEndTask {
	public GamepadDriveTask(MecanumSubsystem drivetrain, Gamepad gamepad) {
		super(() -> {
			drivetrain.mecanum().setDriveDST(
					() -> gamepad.left_stick_y,
					() -> gamepad.left_stick_x,
					() -> gamepad.right_stick_x
			);
		}, () -> {
			drivetrain.mecanum().idle();
		}, drivetrain);
	}
}
