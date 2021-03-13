package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shimmier extends Subsystem {

	Servo shimmier;

	private double CLOSE = 0.0;
	private double OPEN = 1.0;

	@Override
	public void initialize(HardwareMap hardwareMap) {
		shimmier = hardwareMap.get(Servo.class, "shimmier");
	}

	@Override
	public void initPeriodic() {

	}

	@Override
	public void start() {
		open();
	}

	@Override
	public void runPeriodic() {

	}

	@Override
	public void stop() {

	}

	public void close() {
		shimmier.setPosition(CLOSE);
	}

	public void open() {
		shimmier.setPosition(OPEN);
	}
}
