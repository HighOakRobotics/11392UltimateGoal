package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class OdometrySensor extends PositioningSensor{

	TwoWheelLocalizer localizer;
	Supplier<TwoWheelLocalizer> localizerSupplier;

	public OdometrySensor(Supplier<TwoWheelLocalizer> localizerSupplier) {
		priority = 0;
		this.localizerSupplier = localizerSupplier;
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		localizer = localizerSupplier.get();
	}

	public Pose2d getVelocity() {
		return localizer.getPoseVelocity();
	}

	@Override
	public void initPeriodic() {
		runPeriodic();
	}

	@Override
	public void runPeriodic() {
		localizer.update();
		telemetry.addData("odox", getPositionSupplier().get().getX());
		telemetry.addData("odoy", getPositionSupplier().get().getY());
		telemetry.addData("odor", getPositionSupplier().get().getHeading());
	}

	@Override
	public Supplier<Position> getPositionSupplier() {
		return localizer.getPositionSupplier();
	}

	@Override
	public Consumer<Position> getPositionReset() {
		return (Position pos) -> {
			telemetry.log().add("reset odo position");
			localizer.setPoseEstimate(new Pose2d(pos.getX(), pos.getY(), pos.getHeading()));
		};
	}
}
