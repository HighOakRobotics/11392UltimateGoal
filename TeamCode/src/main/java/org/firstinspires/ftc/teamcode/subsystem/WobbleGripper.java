package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGripper extends Subsystem {

	Servo finger1;
	Servo finger2;

	public void open() {
		finger1.setPosition(0);
		finger2.setPosition(1);
	}

	public void close() {
		finger1.setPosition(1);
		finger2.setPosition(0);
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		finger1 = hardwareMap.get(Servo.class, "finger1");
		finger2 = hardwareMap.get(Servo.class, "finger2");
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
