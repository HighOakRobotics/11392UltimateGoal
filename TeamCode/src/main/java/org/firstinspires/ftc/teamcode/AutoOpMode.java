package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.Scheduler;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.SwitchTask;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Loader;
import org.firstinspires.ftc.teamcode.subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.subsystem.RingDetector;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;
import org.firstinspires.ftc.teamcode.subsystem.WobbleGripper;
import org.firstinspires.ftc.teamcode.task.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.task.LoaderPushTask;
import org.firstinspires.ftc.teamcode.task.RingDetectTask;
import org.firstinspires.ftc.teamcode.task.StartShooterTask;
import org.firstinspires.ftc.teamcode.task.StopShooterTask;
import org.firstinspires.ftc.teamcode.task.TiltModeSelectTask;
import org.firstinspires.ftc.teamcode.task.WobbleGripperControlTask;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoOpMode 11392", group = "11392", preselectTeleOp = "DriveOpMode 11392")
@Disabled
public class AutoOpMode extends SequoiaOpMode {
	private final Lift lift = new Lift();
	private final Tilt tilt = new Tilt();
	private final Mecanum mecanum = new Mecanum();
	private final WobbleGripper gripper = new WobbleGripper();
	private final Shooter shooter = new Shooter();
	private final Loader loader = new Loader();
	private final RingDetector ringDetector = new RingDetector();

	// SampleMecanumDrive drive;
	private int boxX = -1;
	private int boxY = -1;
	private double boxHeading = -1;

	private final HashMap<Object, Task> poseMappings = new HashMap<Object, Task>(){{
		put(0, new InstantTask(() -> {
			boxX = 12;
			boxY = -52;
			boxHeading = 0;
		}));
		put(1, new InstantTask(() -> {
			boxX = 36;
			boxY = -44;
			boxHeading = Math.PI;
		}));
		put(4, new InstantTask(() -> {
			boxX = 52;
			boxY = -52;
			boxHeading = 0;
		}));
	}};

	@Override
	public void initTriggers() {
		Scheduler.getInstance().schedule(new RingDetectTask(ringDetector));
		mecanum.mecanum().setPoseEstimate(new Pose2d(-63, -48, Math.PI));
	}

	@Override
	public void runTriggers() {
		Scheduler.getInstance().schedule(new SequentialTaskBundle(
				new SwitchTask(poseMappings, ringDetector::getDetectedRings),
				new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.CLOSE, gripper),
				new WaitTask(500, TimeUnit.MILLISECONDS),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToConstantHeading(new Vector2d(-24, -58))
								.build()
				),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToLinearHeading(new Pose2d(boxX, boxY, boxHeading)).build()
				),
				new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.OPEN, gripper),
				new WaitTask(500, TimeUnit.MILLISECONDS),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToConstantHeading(new Vector2d(-24, -58))
								.build()
				),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToLinearHeading(new Pose2d(-48, -48, Math.PI))
								.build()
				),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToConstantHeading(new Vector2d(-48, -36))
								.build()
				),
				new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.CLOSE, gripper),
				new WaitTask(500, TimeUnit.MILLISECONDS),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToConstantHeading(new Vector2d(-24, -58))
								.build()
				),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToLinearHeading(new Pose2d(boxX - 6, boxY, boxHeading)).build()
				),
				new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.OPEN, gripper),
				new WaitTask(500, TimeUnit.MILLISECONDS),
				new StartShooterTask(shooter),
				new TiltModeSelectTask(TiltModeSelectTask.Position.SHOOT, tilt),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToLinearHeading(new Pose2d(-6, -42, Math.PI))
								.build()
				),
				new LoaderPushTask(loader),
				new WaitTask(400, TimeUnit.MILLISECONDS),
				new LoaderPushTask(loader),
				new WaitTask(400, TimeUnit.MILLISECONDS),
				new LoaderPushTask(loader),
				new WaitTask(400, TimeUnit.MILLISECONDS),
				new StopShooterTask(shooter),
				new TiltModeSelectTask(TiltModeSelectTask.Position.BASE, tilt),
				new FollowTrajectoryTask(
						mecanum,
						() -> mecanum.mecanum().trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
								.lineToConstantHeading(new Vector2d(10, -42))
								.build()
				),
				new InstantTask(this::requestOpModeStop)
		));
	}

}