package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem;

public class GamepadDriveTask extends StartEndTask {
	public GamepadDriveTask(MecanumSubsystem drivetrain, Gamepad gamepad) {
		super(() -> {
			drivetrain.mecanum().setDriveDST(
					() -> gamepad.left_stick_y * Math.abs(gamepad.left_stick_y),
					() -> gamepad.left_stick_x * Math.abs(gamepad.left_stick_x),
					() -> gamepad.right_stick_x * Math.abs(gamepad.right_stick_x)
			);
		}, () -> {
			drivetrain.mecanum().idle();
		}, drivetrain);
	}
}
