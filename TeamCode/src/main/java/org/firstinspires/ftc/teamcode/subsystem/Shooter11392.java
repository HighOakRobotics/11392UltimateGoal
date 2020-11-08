package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter11392 extends Subsystem {

	private final double INIT_TRACK_ANGLE = 0.0;
	private final double INIT_SHOOTER_PITCH = 0.0;
	private final double MAX_VELOCITY = 6000; // ?? need to tune
	private DcMotorEx flywheel;

	private double flywheelVelocity; // in RPM
	private double trackAngle;
	private double shooterPitch;

	public double getFlywheelVelocity() {
		return flywheelVelocity;
	}

	public void setFlywheelVelocity(double flywheelVelocity) {
		this.flywheelVelocity = flywheelVelocity;
	}

	public double getTrackAngle() {
		return trackAngle;
	}

	public void setTrackAngle(double trackAngle) {
		this.trackAngle = trackAngle;
	}

	public double getShooterPitch() {
		return shooterPitch;
	}

	public void setShooterPitch(double shooterPitch) {
		this.shooterPitch = shooterPitch;
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

		flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		//flywheel.setMotorType();
		flywheel.setVelocity(0); // In encoder mode it runs at a fraction of maximum velocity.
	}

	@Override
	public void initPeriodic() {
	}

	@Override
	public void start() {
	}

	@Override
	public void runPeriodic() {
		flywheel.setVelocity(flywheelVelocity);
		telemetry.addLine()
				.addData("flywheel ", flywheelVelocity);
	}

	@Override
	public void stop() { flywheel.setVelocity(0);
	}
}
