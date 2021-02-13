package org.firstinspires.ftc.teamcode.legacy.advancednav;

public class Trajectory {
	private double startX;
	private double endX;
	//private PolynomialSplineFunction splint;

	public Trajectory(double startX, double endX/*, PolynomialSplineFunction splint*/) {
		this.startX = startX;
		this.endX = endX;
		//this.splint = splint;
	}

	public Trajectory(Positron[] trajectory, boolean inverted) {
		//SplineInterpolator splint = new SplineInterpolator();
		double[] xPos = new double[trajectory.length];
		double[] yPos = new double[trajectory.length];

		for (int i = 0; i < trajectory.length; i++) {
			xPos[i] = trajectory[i].getX();
			yPos[i] = trajectory[i].getY();
		}

		//if inverted is true then the robot needs to go vertical at some point
		//if the robot both needs to go
		if (!inverted) {
			startX = xPos[0];
			endX = xPos[trajectory.length - 1];
			//this.splint = splint.interpolate(xPos, yPos);
		} else {
			startX = yPos[0];
			endX = yPos[trajectory.length - 1];
			//this.splint = splint.interpolate(yPos, xPos);
		}
	}

	public Trajectory(Positron[] trajectory) {
		this(trajectory, false);
	}

	public double getStartX() {
		return startX;
	}

	public double getEndX() {
		return endX;
	}

    /*
    public PolynomialSplineFunction getSpline() {
        return splint;
    }
    public static Trajectory generateTrajectory(Positron[] trajectory, boolean inverted){
        SplineInterpolator splint = new SplineInterpolator();
        double[] xPos = new double[trajectory.length];
        double[] yPos = new double[trajectory.length];
        for(int i = 0; i < trajectory.length; i++){
            xPos[i] = trajectory[i].getX();
            yPos[i] = trajectory[i].getY();
        }
        //if inverted is true then the robot needs to go vertical at some point
        //if the robot both needs to go
        if(!inverted)
            return new Trajectory(xPos[0], xPos[trajectory.length - 1], splint.interpolate(xPos, yPos));
        else
            return new Trajectory(yPos[0], yPos[trajectory.length - 1], splint.interpolate(yPos, xPos));
    }
     */
}
