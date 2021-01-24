package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter extends Subsystem {

	private final double INIT_TRACK_ANGLE = 0.0;
	private final double INIT_SHOOTER_PITCH = 0.0;
	private final double MAX_VELOCITY = 5000; // ?? need to tune
	private DcMotorEx flywheel;

	private double flywheelVelocity; // in RPM
	private double trackAngle;
	private double shooterPitch;

	public double getDesiredFlywheelVelocity() {
		return flywheelVelocity;
	}

	public void setDesiredFlywheelVelocity(double flywheelVelocity) {
		this.flywheelVelocity = flywheelVelocity;
	}

	public double getFlywheelVelocity() {
		return flywheel.getVelocity();
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
		flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15,3,0,0));
		PIDFCoefficients pidf = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
		telemetry.log().add("Shooter PID: P:"+pidf.p+" I:"+pidf.i+" D:"+pidf.d+" F:"+pidf.f);
		flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
		//flywheel.setMotorType();
		flywheel.setVelocity(0);
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
		telemetry.addData("flywheel ", flywheel.getVelocity());
	}

	@Override
	public void stop() {
		flywheel.setVelocity(0);
	}
}
