package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;

import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;

public class LiftControlTask extends InstantTask {
	public LiftControlTask(int adjustment, WobbleArm wobbleArm) {
		super(() -> {
			wobbleArm.modifyTarget(adjustment);
		});
	}
}
