package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Tilt extends Subsystem {

	private static final int MIN_POSITION = 0;
	private static final int MAX_POSITION = 500;
	private static final double RUN_POWER = 0.9;
	public int offset = 0;
	public int targetPosition = 0;
	private DcMotorEx tilt;
	private double tiltPower;

	public double getTiltPower() {
		return tiltPower;
	}

	public void setTiltPower(double tiltPower) {
		this.tiltPower = tiltPower;
	}

	public int getTargetPosition() {
		return targetPosition;
	}

	public void setTargetPosition(int targetPosition) {
		this.targetPosition = targetPosition;
	}

	private void setMotorTarget(int position) {
		int desiredPosition = offset + position;
		tilt.setTargetPosition(Range.clip(desiredPosition, MIN_POSITION, MAX_POSITION));
	}

	public void runToZero() {
		tilt.setDirection(DcMotorSimple.Direction.REVERSE);
		tilt.setPower(-0.2);
		tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	public void reset() {
		tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		offset = tilt.getCurrentPosition();
		tilt.setTargetPosition(offset);
		tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		tilt.setPower(RUN_POWER);
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		tilt = hardwareMap.get(DcMotorEx.class, "tilt");
		tilt.setDirection(DcMotorSimple.Direction.REVERSE);
		tilt.setPower(-0.2);
		tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	@Override
	public void initPeriodic() {
	}

	@Override
	public void start() {
		reset();
	}

	@Override
	public void runPeriodic() {
		setMotorTarget(targetPosition);
		telemetry.addData("tilt", tilt.getCurrentPosition() - offset);
	}

	@Override
	public void stop() {
		tilt.setPower(0);
	}
}
