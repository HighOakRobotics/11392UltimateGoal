package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class LiftControlTask extends InstantTask {
	public LiftControlTask(int adjustment, Lift lift) {
		super(() -> {
			lift.modifyTarget(adjustment);
		});
	}
}
