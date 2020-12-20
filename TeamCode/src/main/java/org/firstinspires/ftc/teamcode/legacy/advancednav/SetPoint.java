package org.firstinspires.ftc.teamcode.legacy.advancednav;

public class SetPoint{
	private double velo;
	private double dist;
	public SetPoint(double velo, double dist){
		this.velo = velo;
		this.dist = dist;
	}

	public double getVelo() {
		return velo;
	}

	public double getDist() {
		return dist;
	}
}