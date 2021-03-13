package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystem.Tilt;

public class ResetTiltTask extends Task {
	private final Tilt tilt;
	private final Clock clock;

	public ResetTiltTask(Tilt tilt) {
		this.clock = new Clock();
		this.tilt = tilt;
	}


	@Override
	public void init() {
		clock.startTiming();
		tilt.runToZero();
	}

	@Override
	public void loop() {
		if(clock.getMillis() > 1500) {
			this.running = false;
		}
	}

	@Override
	public void stop(boolean interrupted) {
		tilt.reset();
	}
}
