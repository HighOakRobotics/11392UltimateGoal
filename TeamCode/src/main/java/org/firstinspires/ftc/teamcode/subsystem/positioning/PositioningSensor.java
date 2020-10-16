package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

public abstract class PositioningSensor extends Subsystem {
	Position position;

	public PositioningSensor() {
		this(new Position());
	}

	public PositioningSensor(Position initialPosition) {
		position = initialPosition;
	}

	public abstract Supplier<Position> getPositionSupplier();

	@Override
	public void initialize(HardwareMap hardwareMap) {
		priority = 0;
	}

	@Override
	public void start() {
	}

	@Override
	public void initPeriodic() {
	}

	@Override
	public void runPeriodic() {
	}

	@Override
	public void stop() {
	}
}
