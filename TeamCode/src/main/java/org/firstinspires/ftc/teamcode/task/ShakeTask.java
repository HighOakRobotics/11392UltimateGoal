package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;

public class ShakeTask extends Task {

	Tilt tilt;
	Intake intake;
	Shooter shooter;
	Clock clock;

	public ShakeTask(Tilt tilt, Intake intake) {
		this.tilt = tilt;
		this.intake = intake;
	}

	@Override
	public void init() {
		tilt.setTargetPosition(TiltModeSelectTask.Position.SHAKE.pos());
		intake.setIntakePower(1.0);
		clock = new Clock();
		clock.startTiming();
		this.running = true;
	}

	@Override
	public void loop() {
		if (clock.getMillis() >= 1000)
			running = false;
	}

	@Override
	public void stop(boolean interrupted) {
		intake.setIntakePower(0);
		tilt.setTargetPosition(TiltModeSelectTask.Position.SHOOT.pos());
	}
}
