package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.RingDetector;

public class RingDetectTask extends Task {
	RingDetector detector;

	public RingDetectTask(RingDetector detector) {
		this.running = true;
		this.detector = detector;

		addSubsystems(detector);
	}

	@Override
	public void init() {
	}

	@Override
	public void loop() {
		detector.detectRings();
	}

	@Override
	public void stop(boolean interrupted) {
		detector.stop();
		this.running = false;
	}
}
