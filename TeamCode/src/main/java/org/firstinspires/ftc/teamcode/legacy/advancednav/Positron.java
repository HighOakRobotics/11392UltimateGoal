package org.firstinspires.ftc.teamcode.legacy.advancednav;

public class Positron {
	private double x; //Inches
	private double y; //Inches
	private double heading; //Degrees, 0-360 starting from top and going clockwise

	public Positron(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getHeading() {
		return heading;
	}
}
