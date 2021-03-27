package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc11392.sequoia.task.PIDFTask;
import com.ftc11392.sequoia.util.PIDFController;

import org.firstinspires.ftc.teamcode.subsystem.Mecanum;

import java.util.function.DoubleConsumer;

public class PointControlTask extends PIDFTask {

	double turn = 0;

	public PointControlTask(Mecanum drivetrain, DoubleConsumer output) {
		super(new PIDFController(0.5, 0, 0),
				() -> {
					double rarrad =  drivetrain.mecanum().getPoseEstimate().getHeading() + Math.PI;
					if (rarrad < Math.PI) rarrad += Math.PI * 2;
					return rarrad;
				},
				() -> {
					Pose2d pos = drivetrain.mecanum().getPoseEstimate();
					double rad = Math.atan2(72-pos.getX(), 36+pos.getY());
					double trurad = rad + 3*Math.PI/2;
					return trurad;
				},
				output);
	}
}

