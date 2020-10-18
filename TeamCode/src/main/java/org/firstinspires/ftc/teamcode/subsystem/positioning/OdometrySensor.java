package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

public class OdometrySensor extends PositioningSensor {

	public StandardTrackingWheelLocalizer odometry;
	private Pose2d rrPoseEstimate;
	private Position position;

	private void update() {
		odometry.update();
		rrPoseEstimate = odometry.getPoseEstimate();
		position = new Position(rrPoseEstimate.getX(), rrPoseEstimate.getY(), rrPoseEstimate.getHeading());
	}

	@Override
	public Supplier<Position> getPositionSupplier() {
		return () -> position;
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		super.initialize(hardwareMap);
		odometry = new StandardTrackingWheelLocalizer(hardwareMap);
		update();
	}

	@Override
	public void runPeriodic() {
		update();
	}

	@Override
	public void initPeriodic() {
		update();
	}
}
