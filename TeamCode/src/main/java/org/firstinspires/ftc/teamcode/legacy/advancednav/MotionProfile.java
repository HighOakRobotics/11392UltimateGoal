package org.firstinspires.ftc.teamcode.legacy.advancednav;

public class MotionProfile {
	//Stage 1: Acceleration Increasing To Max
	//Stage 2: Acceleration Constant At Max
	//Stage 3: Acceleration Decreasing To 0
	//Stage 4: Cruising (0 acceleration)
	//Stage 5: Deceleration Increasing To Max
	//Stage 6: Deceleration Constant At Max
	//Stage 7: Deceleration Decreasing To 0
	private double[] stageDurations;
	private double maxAccel;
	private double maxVelo;
	private double[] defaultStageDurations = {0.25, 0.5, 0.25, 0, 0.25, 0.5, 0.25};

	public MotionProfile(double[] stageDurations, double maxPower) {
		this.stageDurations = stageDurations;
		maxVelo = maxPower;
		maxAccel = maxVelo / (stageDurations[1] + stageDurations[0]);
	}

	public MotionProfile(double distance, double maxPower) {
		stageDurations = defaultStageDurations;
		distance -= getTotalDistance();
		stageDurations[3] = distance / maxPower;
		maxVelo = maxPower;
		maxAccel = maxVelo / (stageDurations[1] + stageDurations[0]);
	}

	public SetPoint lookUpSetpoint(double time) {
		return new SetPoint(lookUpVelocity(time), lookUpDistance(time));
	}

	public double getTotalDistance() {
		return lookUpDistance(totalStageDuration());
	}

	public double lookUpDistance(double time) {
		return lookUpDistance(time, 0.05);
	}

	public double lookUpDistance(double time, double iter) {
		double distCounter = 0;
		for (int i = 0; i < time; i += iter) {
			distCounter += lookUpVelocity(i) * iter;
		}
		return distCounter;
	}

	public double lookUpVelocity(double time) {
		return lookUpVelocity(time, 0.05);
	}

	public double lookUpVelocity(double time, double iter) {
		double veloCounter = 0;
		for (int i = 0; i < time; i += iter) {
			veloCounter += lookUpAcceleration(i) * iter;
		}
		return veloCounter;
	}

	public double lookUpAcceleration(double time) {
		switch (whichStage(time)) {
			case 0:
				return (time / stageDurations[0]) * maxAccel;
			case 1:
			case 5:
				return maxAccel;
			case 2:
				return maxAccel - (((time - totalStageDurationBefore(2)) / stageDurations[2]) * maxAccel);
			case 3:
				return 0;
			case 4:
				return -(((time - totalStageDurationBefore(4)) / stageDurations[4]) * maxAccel);
			case 6:
				return (((time - totalStageDurationBefore(6)) / stageDurations[6]) * maxAccel) - maxAccel;
			default:
				return 0;
		}
	}

	/**
	 * Figure out which stage of acceleration / deceleration we're in. Returns -1
	 * if the time is outside of the stages times.
	 *
	 * @param time
	 * @return
	 */
	private int whichStage(double time) {
		for (int i = 0; i < 7; i++) {
			if (stageDurations[i] > time) {
				return i;
			}
			time -= stageDurations[i];
		}
		return -1;
	}

	public double totalStageDuration() {
		int counter = 0;
		for (double cur : stageDurations) {
			counter += cur;
		}
		return counter;
	}

	private double totalStageDurationBefore(int stageId) {
		double counter = 0;
		for (int i = 0; i < stageId; i++) {
			counter += stageDurations[i];
		}
		return counter;
	}

}