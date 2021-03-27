package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Loader extends Subsystem {

	public final double LOADER_CLOSE = 0.5;
	public final double LOADER_OPEN = 0.0;
	Servo loader;
	boolean loaderState;

	@Override
	public void initialize(HardwareMap hardwareMap) {
		loader = hardwareMap.get(Servo.class, "loader");
		loader.setPosition(LOADER_OPEN);
		loaderState = true ;
	}

	public void update() {
		if (loaderState)
			loader.setPosition(LOADER_OPEN);
		else
			loader.setPosition(LOADER_CLOSE);
	}

	@Override
	public void initPeriodic() {
		update();
	}

	@Override
	public void start() {
	}

	@Override
	public void runPeriodic() {
		update();
	}

	@Override
	public void stop() {
	}

	public void setLoaderState(LoaderState a) {
		if (a == LoaderState.OPEN)
			loaderState = true;
		else if (a == LoaderState.CLOSED)
			loaderState = false;
	}

	public enum LoaderState {
		CLOSED,
		OPEN
	}
}
