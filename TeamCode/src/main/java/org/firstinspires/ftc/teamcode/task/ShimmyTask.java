package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystem.Shimmier;

public class ShimmyTask extends Task {

	Shimmier shimmier;
	Clock clock;

	public ShimmyTask(Shimmier shimmier) {
		this.shimmier = shimmier;
		clock = new Clock();
	}

	@Override
	public void init() {
		shimmier.close();
		clock.startTiming();
	}

	@Override
	public void loop() {
		if (clock.getMillis() > 500)
			running = false;
	}

	@Override
	public void stop(boolean interrupted) {
		shimmier.open();

	}
}
