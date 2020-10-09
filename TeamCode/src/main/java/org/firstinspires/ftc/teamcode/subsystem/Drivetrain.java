package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Drivetrain extends Subsystem {

	public enum driveState {

	}

	List<DcMotorEx> motors = new ArrayList<>();
	DcMotorEx frontLeft, frontRight, backLeft, backRight;

	@Override
	public void initialize(HardwareMap hardwareMap) {
		frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
		frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
		backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
		backRight = hardwareMap.get(DcMotorEx.class, "backRight");

		motors.addAll(Arrays.asList(frontLeft, frontRight, backLeft, backRight));

		for (DcMotorEx motor : motors) {
			motor.setDirection(DcMotorSimple.Direction.FORWARD);
			motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		}
	}

	public void dstDrive(double d, double s, double t) {
		// TODO ACTUALLY DO THIS
	}

	@Override
	public void runPeriodic() { }

	@Override
	public void initPeriodic() { }

	@Override
	public void stop() {
		for (DcMotorEx motor : motors) {
			motor.setPower(0);
		}
	}
}