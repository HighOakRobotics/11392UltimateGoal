package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.positioning.Position;
import org.firstinspires.ftc.teamcode.subsystem.positioning.PositionLocalizer;

import java.util.function.Supplier;

public class MecanumSubsystem extends Subsystem {
	DriveTrainMecanum mecanum;
	Supplier<Position> positionSupplier;

	public MecanumSubsystem(Supplier<Position> positionSupplier) {
		this.positionSupplier = positionSupplier;
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		mecanum = new DriveTrainMecanum(hardwareMap);
		mecanum.setLocalizer(new PositionLocalizer(positionSupplier));
		mecanum.setMotorPowers(0, 0, 0, 0);
		mecanum.update();
	}

	@Override
	public void initPeriodic() {
		mecanum.update();
	}

	@Override
	public void start() {
	}

	@Override
	public void runPeriodic() {
		mecanum.update();
	}

	@Override
	public void stop() {
		mecanum.setMotorPowers(0, 0, 0, 0);
		mecanum.update();
	}

	public DriveTrainMecanum mecanum() {
		return mecanum;
	}
}
