package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class StopShooterTask extends InstantTask {

	public StopShooterTask(Shooter shooter) {
		super(() -> shooter.setDesiredFlywheelVelocity(0));
	}
}

