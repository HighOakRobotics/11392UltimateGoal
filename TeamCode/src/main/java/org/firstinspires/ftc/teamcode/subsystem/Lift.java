package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Lift extends Subsystem {

	private static final int MIN_POSITION = 0;
	private static final int MAX_POSITION = 1150;
	private static final double RUN_POWER = 0.8;
	public int offset = 0;
	public int targetPosition = 0;
	private DcMotorEx lift;
	private double liftPower;

	public double getLiftPower() {
		return liftPower;
	}

	public void setLiftPower(double liftPower) {
		this.liftPower = liftPower;
	}

	public int getTargetPosition() {
		return targetPosition;
	}

	public void setTargetPosition(int targetPosition) {
		this.targetPosition = targetPosition;
	}

	public void modifyTarget(int tool) {
		targetPosition = Range.clip(targetPosition + tool, MIN_POSITION, MAX_POSITION);
	}

	private void setMotorTarget(int position) {
		int desiredPosition = offset + position;
		lift.setTargetPosition(Range.clip(desiredPosition, MIN_POSITION, MAX_POSITION));
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		lift = hardwareMap.get(DcMotorEx.class, "lift");
		lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		offset = lift.getCurrentPosition();
		lift.setTargetPosition(offset);
		lift.setPower(0);
		lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	@Override
	public void initPeriodic() {
	}

	@Override
	public void start() {
		lift.setPower(RUN_POWER);
	}

	@Override
	public void runPeriodic() {
		setMotorTarget(targetPosition);
		telemetry.addData("lift", lift.getCurrentPosition() - offset);
	}

	@Override
	public void stop() {
		lift.setPower(0);
	}
}
