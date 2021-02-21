package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.task.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.legacy.HolyExt;
import org.firstinspires.ftc.teamcode.subsystem.DriveConstants;

@Autonomous
@Disabled
public class RTTestOpMode extends LinearOpMode {
	private HolyExt holy;
	private double TURN_KMI = 0.08;
	private double TURN_KP = 0.06;
	private double TURN_KI = 0.0018;
	private double TURN_KD = 2.5;
	private double TURN_KF = 0.0;

	@Override
	public void runOpMode() throws InterruptedException {
		resetScheduler();

		holy = new HolyExt(hardwareMap, telemetry, 1, 1, 1, true, this);
		holy.init();
		holy.resetEncoder();
		holy.initRT(0.8);
		holy.turnOffBrakes();
		holy.setTPR(DriveConstants.TICKS_PER_REV);
		holy.setMaxI(TURN_KMI);
		holy.setPIDTurnCoefficients(new PIDFCoefficients(TURN_KP, TURN_KI, TURN_KD, TURN_KF));

		waitForStart();

		holy.driveByInchSynchRT(10, 0.6, this);
		holy.turnPIDAbsoluteSynch(0, 0.5);
		holy.driveByInchSynchRT(10, 0.6, this);
	}

	private void resetScheduler() {
		Scheduler.getInstance().cancelAll();
		Scheduler.getInstance().clearBehaviors();
		Scheduler.getInstance().clearSubsystems();
	}
}
