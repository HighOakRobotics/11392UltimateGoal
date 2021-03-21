package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class VSLAMSensor extends PositioningSensor {

	private static T265Camera slamra = null;

	private Position initial;
	private boolean selfUpdate;

	public VSLAMSensor(Position initial, boolean selfUpdate) {
		this.initial = initial;
		this.selfUpdate = selfUpdate;
	}

	public VSLAMSensor(Position initial) {
		this(initial, false);
	}

	public VSLAMSensor() {
		this(new Position(0.0,0.0,0.0));
	}

	@Override
	public Supplier<Position> getPositionSupplier() {
		return this::update;
	}

	@Override
	public Consumer<Position> getPositionReset() {
		return (Position pos) -> {
			if (pos.hasAll()) {
				slamra.setPose(new Pose2d(pos.getX(), pos.getY(), new Rotation2d(pos.getHeading())));
			} else {
				throw new IllegalArgumentException("Position reset must contain all fields for this sensor,");
			}
		};
	}

	private Position update() {
		T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
		if (up == null) {
			return new Position(null,null,null);
		}

		// We divide by 0.0254 to convert meters to inches
		Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
		Rotation2d rotation = up.pose.getRotation();

		return new Position(translation.getX(), translation.getY(), rotation.getRadians());
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		priority = 0; // Probably not necessary
		if (slamra == null) {
			slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
		}
	}

	@Override
	public void start() {
		slamra.start();
		slamra.setPose(new Pose2d(0,0,new Rotation2d()));
	}

	@Override
	public void runPeriodic() {
		if (selfUpdate) {
			Position curPos = update();
			if (curPos.hasX()) telemetry.addData("x", curPos.getX());
			if (curPos.hasY()) telemetry.addData("y", curPos.getY());
			if (curPos.hasHeading()) telemetry.addData("r", curPos.getHeading());
		}
	}

	@Override
	public void stop() {
		slamra.stop();
	}
}