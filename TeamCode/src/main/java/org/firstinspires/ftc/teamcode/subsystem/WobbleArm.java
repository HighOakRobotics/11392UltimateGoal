package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class WobbleArm extends Subsystem {

	private static final int MIN_POSITION = 0;
	private static final int MAX_POSITION = 1150;
	private static final double RUN_POWER = 1.0;
	public int offset = 0;
	public int targetPosition = 0;
	private DcMotorEx armMotor;
	private double wobbleArmPower;

	public double getWobbleArmPower() {
		return wobbleArmPower;
	}

	public void setWobbleArmPower(double wobbleArmPower) {
		this.wobbleArmPower = wobbleArmPower;
	}

	public int getTargetPosition() {
		return targetPosition;
	}

	private void setTargetPosition(int targetPosition) {
		this.targetPosition = targetPosition;
	}

	public void modifyTarget(int tool) {
		targetPosition = Range.clip(targetPosition + tool, MIN_POSITION, MAX_POSITION);
	}

	public void setMotorTarget(int position) {
		int desiredPosition = offset + position;
		targetPosition = (Range.clip(desiredPosition, MIN_POSITION, MAX_POSITION));
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		armMotor = hardwareMap.get(DcMotorEx.class, "wobble");
		armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		offset = armMotor.getCurrentPosition();
		armMotor.setTargetPosition(offset);
		armMotor.setPower(0);
		armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	@Override
	public void initPeriodic() {
	}

	@Override
	public void start() {
		armMotor.setPower(RUN_POWER);
	}

	@Override
	public void runPeriodic() {
		armMotor.setTargetPosition(targetPosition);
		telemetry.addData("wobble", armMotor.getCurrentPosition() - offset);
		telemetry.addData("wobbleTarget", targetPosition);
	}

	@Override
	public void stop() {
		armMotor.setPower(0);
	}
}
