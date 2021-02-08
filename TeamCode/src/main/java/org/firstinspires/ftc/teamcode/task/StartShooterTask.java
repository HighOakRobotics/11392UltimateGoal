package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class StartShooterTask extends InstantTask {

	public StartShooterTask(Shooter shooter) {
		super(() -> shooter.setDesiredFlywheelVelocity(1500));
	}
}

