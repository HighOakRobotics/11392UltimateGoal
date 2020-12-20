package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GenUtil {
	public static void sleep(double milliseconds, LinearOpMode justPutThis){
		ElapsedTime et = new ElapsedTime();
		et.reset();
		while(et.milliseconds() < milliseconds && !justPutThis.isStopRequested() && justPutThis.opModeIsActive()){

		}
	}
}