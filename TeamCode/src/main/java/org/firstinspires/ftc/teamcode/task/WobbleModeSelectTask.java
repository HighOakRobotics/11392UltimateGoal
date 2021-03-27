package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;

import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;

public class WobbleModeSelectTask extends InstantTask {

	public WobbleModeSelectTask(Position position, WobbleArm arm) {
		super(() -> arm.setMotorTarget(position.pos()));
	}

	public enum Position {
		INIT(0),
		START(20),
		HOLD(30),
		GRAB(40);

		Position(int pos) {
			position = pos;
		}

		private final int position;

		public int pos() {
			return position;
		}
	}
}
