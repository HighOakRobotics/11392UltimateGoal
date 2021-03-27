package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc11392.sequoia.task.PIDFTask;
import com.ftc11392.sequoia.util.PIDFController;

import org.firstinspires.ftc.teamcode.subsystem.Mecanum;

import java.util.function.DoubleConsumer;

public class PointControlTask extends PIDFTask {

	double turn = 0;

	public PointControlTask(Mecanum drivetrain, DoubleConsumer output) {
		super(new PIDFController(1, 0, 0),
				() -> drivetrain.mecanum().getPoseEstimate().getHeading(),
				() -> {
					Pose2d pos = drivetrain.mecanum().getPoseEstimate();
					return Math.PI - Math.atan2(pos.getY() - 72, pos.getX() + 36);
				},
				output);
	}
}
