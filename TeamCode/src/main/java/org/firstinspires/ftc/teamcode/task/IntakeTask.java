package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class IntakeTask extends Task {
	Intake intake;

	public IntakeTask(Intake intake) {
		this.running = true;
		this.intake = intake;
		addSubsystems(intake);
	}

	@Override
	public void init() {
		intake.setIntakePower(-0.7);
	}

	@Override
	public void loop() {
		intake.setIntakePower(-0.7);
	}

	@Override
	public void stop(boolean interrupted) {
		intake.setIntakePower(0.0);
	}

}
