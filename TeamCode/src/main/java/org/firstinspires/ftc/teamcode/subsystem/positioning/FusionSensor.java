package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class FusionSensor extends PositioningSensor {

	VSLAMSensor vslam;
	OdometrySensor odo;

	public FusionSensor(VSLAMSensor vslam, OdometrySensor odo) {
		this.vslam = vslam;
		this.odo = odo;
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		telemetry.log().add("vslam odo application position x:" + vslam.getPositionSupplier().get().getX());
		odo.getPositionReset().accept(vslam.getPositionSupplier().get());
	}

	@Override
	public void initPeriodic() {
		runPeriodic();
	}

	@Override
	public void runPeriodic() {
		Pose2d vels = odo.getVelocity();
		if (vels!=null) {
			telemetry.addData("odovels", "exists");
			vslam.getT265Camera().sendOdometry(vels.getX()*0.0254, vels.getY()*0.0254);
		} else {
			telemetry.addData("odovels", "do not exist");
		}

	}

	@Override
	public Supplier<Position> getPositionSupplier() {
		return vslam.getPositionSupplier();
	}

	@Override
	public Consumer<Position> getPositionReset() {
		return (Position pos) -> {
			odo.getPositionReset().accept(pos);
			vslam.getPositionReset().accept(pos);
		};
	}
}
