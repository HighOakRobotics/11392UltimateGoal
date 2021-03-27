package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Blockers extends Subsystem {
	Servo blockerLeft;
	Servo blockerRight;

	public void open() {
		blockerLeft.setPosition(1);
		blockerRight.setPosition(0);
	}

	public void close() {
		blockerLeft.setPosition(0);
		blockerRight.setPosition(0.9);
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		blockerLeft = hardwareMap.get(Servo.class, "blockerLeft");
		blockerRight = hardwareMap.get(Servo.class, "blockerRight");
		close();
	}

	@Override
	public void initPeriodic() {

	}

	@Override
	public void start() {

	}

	@Override
	public void runPeriodic() {

	}

	@Override
	public void stop() {

	}
}

