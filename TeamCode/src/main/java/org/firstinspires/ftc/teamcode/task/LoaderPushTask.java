package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystem.Loader;

public class LoaderPushTask extends Task {

	Loader loader;
	Clock clock;

	public LoaderPushTask(Loader loader) {
		this.loader = loader;
		this.clock = new Clock();
	}

	@Override
	public void init() {
		loader.setLoaderState(Loader.LoaderState.CLOSED);
		clock.startTiming();
		this.running = true;
	}

	@Override
	public void loop() {
		if(clock.getMillis() > 300) {
			this.running = false;
		}
	}

	@Override
	public void stop(boolean interrupted) {

		loader.setLoaderState(Loader.LoaderState.OPEN);
	}
}
