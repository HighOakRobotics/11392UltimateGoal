package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Tilt extends Subsystem {

	private static final int MIN_POSITION = 0;
	private static final int MAX_POSITION = 300;
	private static final double RUN_POWER = 0.8;
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

	@Override
	public void initialize(HardwareMap hardwareMap) {
		tilt = hardwareMap.get(DcMotorEx.class, "tilt");
		tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		offset = tilt.getCurrentPosition();
		tilt.setTargetPosition(offset);
		tilt.setPower(0);
		tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	@Override
	public void initPeriodic() {
	}

	@Override
	public void start() {
		tilt.setPower(RUN_POWER);
	}

	@Override
	public void runPeriodic() {
		setMotorTarget(targetPosition);
	}

	@Override
	public void stop() {
		tilt.setPower(0);
	}
}