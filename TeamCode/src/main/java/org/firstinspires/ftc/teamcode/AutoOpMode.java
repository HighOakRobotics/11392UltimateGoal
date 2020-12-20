package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.legacy.HolyExt;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrainMecanum;

@Autonomous(name = "AutoOpMode 11392", group = "11392", preselectTeleOp="DriveOpMode 11392")
public class AutoOpMode extends LinearOpMode {
	HolyExt holy = new HolyExt(hardwareMap, telemetry, this);

	@Override
	public void runOpMode() throws InterruptedException {

		waitForStart();

	}
}
