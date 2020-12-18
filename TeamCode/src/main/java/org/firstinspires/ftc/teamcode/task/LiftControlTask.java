package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.util.Clock;

public class LiftControlTask extends InstantTask {
	Clock limiter = new Clock();
	int minimum = 30; //ms

	public LiftControlTask() {
		super(() -> {
			if (limiter.getMillis() >= minimum) {

			}
		});
		limiter.startTiming();
	}
}
