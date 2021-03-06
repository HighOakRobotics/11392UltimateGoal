package org.firstinspires.ftc.teamcode.tuning;

import com.ftc11392.sequoia.task.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.RingDetector;

@TeleOp(name = "VisionTest 11392", group = "11392")
@Disabled
public class VisionTest extends LinearOpMode {
	private RingDetector vision;

	private int nRings = 0;

	@Override
	public void runOpMode() {
		resetScheduler();

		vision = new RingDetector();

		Scheduler.getInstance().initSubsystems(hardwareMap);
		Scheduler.getInstance().startSubsystems();

		vision.start();
		while (!isStarted()) {
			//waitForStart();
			nRings = vision.detectRings();
			telemetry.addData("rings: ", "%4d", nRings);
			telemetry.update();
		}
		vision.stop();

		while (opModeIsActive()) {

		}

	}

	private void resetScheduler() {
		Scheduler.getInstance().cancelAll();
		Scheduler.getInstance().clearBehaviors();
		Scheduler.getInstance().clearSubsystems();
	}
}
