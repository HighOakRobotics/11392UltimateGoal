package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends Subsystem {

	private final double INIT_TRACK_ANGLE = 0.0;
	private final double INIT_SHOOTER_PITCH = 0.0;
	private DcMotorEx flywheel;
	private Servo loader;
	private Servo track;
	private Servo pivot;
	private double flywheelVelocity;
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

		// TODO Grab hardware devices
		track.setPosition(INIT_TRACK_ANGLE);
		pivot.setPosition(INIT_SHOOTER_PITCH);

		flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		//flywheel.setMotorType();
		flywheel.setPower(0); // In encoder mode it runs at a fraction of maximum velocity.
	}

	@Override
	public void initPeriodic() {
	}

	@Override
	public void start() {
	}

	@Override
	public void runPeriodic() {
		flywheel.setPower(flywheelVelocity);
		track.setPosition(trackAngle);
		pivot.setPosition(shooterPitch);
	}


	@Override
	public void stop() {
		flywheel.setPower(0);
	}
}
