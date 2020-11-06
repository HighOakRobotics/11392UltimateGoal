package org.firstinspires.ftc.teamcode.subsystem;

public class Constants {
	public static final int TICKS_PER_REV = 28;

	/**
	 * Takes in RPM, outputs ticks per second.
	 */
	public static double rpmToTps(double rpm) {
		return rpm * (TICKS_PER_REV/60.0);
	}
}
