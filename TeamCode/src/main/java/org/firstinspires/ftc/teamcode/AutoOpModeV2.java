package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.ConditionalTask;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.Scheduler;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.SwitchTask;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.Lights;
import org.firstinspires.ftc.teamcode.subsystem.Loader;
import org.firstinspires.ftc.teamcode.subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.subsystem.RingDetector;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;
import org.firstinspires.ftc.teamcode.subsystem.WobbleArm;
import org.firstinspires.ftc.teamcode.subsystem.WobbleGripper;
import org.firstinspires.ftc.teamcode.subsystem.positioning.FusionSensor;
import org.firstinspires.ftc.teamcode.subsystem.positioning.OdometrySensor;
import org.firstinspires.ftc.teamcode.subsystem.positioning.PositionLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.positioning.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.positioning.VSLAMSensor;
import org.firstinspires.ftc.teamcode.task.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.task.LoaderPushTask;
import org.firstinspires.ftc.teamcode.task.RingDetectTask;
import org.firstinspires.ftc.teamcode.task.StartShooterTask;
import org.firstinspires.ftc.teamcode.task.StopShooterTask;
import org.firstinspires.ftc.teamcode.task.TiltModeSelectTask;
import org.firstinspires.ftc.teamcode.task.WobbleGripperControlTask;
import org.firstinspires.ftc.teamcode.task.WobbleModeSelectTask;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

@Autonomous(name = "AutoOpModeV2 11392", group = "11392", preselectTeleOp = "DriveOpMode 11392")
//@Disabled
public class AutoOpModeV2 extends SequoiaOpMode {
	private final WobbleArm wobbleArm = new WobbleArm();
	private final Tilt tilt = new Tilt();
	private final Mecanum drivetrain = new Mecanum();
	VSLAMSensor vslam = new VSLAMSensor();
	OdometrySensor odo = new OdometrySensor(
			() -> new TwoWheelLocalizer(hardwareMap,
					() -> drivetrain.mecanum().getRawExternalHeading(),
					() -> drivetrain.mecanum().getRawHeadingVelocity()
			)
	);
	FusionSensor fuse = new FusionSensor(vslam, odo);
	private final WobbleGripper gripper = new WobbleGripper();
	private final Shooter shooter = new Shooter();
	private final Loader loader = new Loader();
	private final RingDetector ringDetector = new RingDetector();
	private final Lights lights = new Lights();

	// SampleMecanumDrive drive;
	private int boxX = -1;
	private int boxY = -1;
	private double boxHeading = -1;

	private int rings = 0;

	private final HashMap<Object, Task> poseMappings = new HashMap<Object, Task>(){{
		put(0, new InstantTask(() -> {
			boxX = -3;
			boxY = -52;
			boxHeading = -Math.PI / 4;
		}));
		put(1, new InstantTask(() -> {
			boxX = 32;
			boxY = -50;
			boxHeading = Math.PI / 2;
		}));
		put(4, new InstantTask(() -> {
			boxX = 52;
			boxY = -52;
			boxHeading = -Math.PI / 4;
		}));
	}};

	@Override
	public void initTriggers() {
		drivetrain.mecanum().setLocalizer(
				new PositionLocalizer(fuse.getPositionSupplier(), fuse.getPositionReset())
		);
		Scheduler.getInstance().schedule(new RingDetectTask(ringDetector));
		drivetrain.mecanum().setPoseEstimate(new Pose2d(-63, -24, Math.PI));
	}

	@Override
	public void runTriggers() {
		drivetrain.mecanum().setPoseEstimate(new Pose2d(-63, -24, Math.PI));
		Scheduler.getInstance().schedule(new SequentialTaskBundle(
				new InstantTask(lights::ocean),
				// Load in mappings
				new WobbleModeSelectTask(WobbleModeSelectTask.Position.HOLD, wobbleArm),
				new InstantTask(() -> rings = ringDetector.getDetectedRings()),
				new SwitchTask(poseMappings, ringDetector::getDetectedRings),
				closeGripper(),
				shootPowerShots(),
				followTrajectory(() -> new Pose2d(boxX+2, boxY, boxHeading)),
				new WobbleModeSelectTask(WobbleModeSelectTask.Position.GRAB, wobbleArm),
				new WaitTask(900,TimeUnit.MILLISECONDS),
				openGripper(),
				new WobbleModeSelectTask(WobbleModeSelectTask.Position.HOLD, wobbleArm),
				followTrajectory(() -> new Pose2d(-24, -57, Math.PI)),
				new WobbleModeSelectTask(WobbleModeSelectTask.Position.GRAB, wobbleArm),
				followConstantTrajectory(() -> new Vector2d(-36, -48)),
				new WaitTask(200, TimeUnit.MILLISECONDS),
				closeGripper(),
				new WobbleModeSelectTask(WobbleModeSelectTask.Position.HOLD, wobbleArm),
				followConstantTrajectory(() -> new Vector2d(-24, -56)),
				followTrajectory(() -> new Pose2d(boxX-2, boxY + 4, boxHeading)),
				new WobbleModeSelectTask(WobbleModeSelectTask.Position.GRAB, wobbleArm),
				new WaitTask(900,TimeUnit.MILLISECONDS),
				openGripper(),
				new WobbleModeSelectTask(WobbleModeSelectTask.Position.INIT, wobbleArm),
				new ConditionalTask(followConstantTrajectory(() -> new Vector2d(10, -30)),
						new InstantTask(() -> {}), () -> rings == 0),
				new ConditionalTask(followConstantTrajectory(() -> new Vector2d(10, -42)),
						new InstantTask(() -> {}), () -> rings != 0),
				new InstantTask(() -> {
					Pose2d pose = drivetrain.mecanum().getPoseEstimate();
					scheduler.putPersistentData("x", Double.toString(pose.getX()));
					scheduler.putPersistentData("y", Double.toString(pose.getY()));
					scheduler.putPersistentData("rot", Double.toString(pose.getHeading()));
					requestOpModeStop();
				})
		));
	}

	private SequentialTaskBundle shootRings(int ringCount) {
		ArrayList<Task> ringShots = new ArrayList<>();
		for(int i = 0; i < ringCount; i++) {
			ringShots.add(new LoaderPushTask(loader));
			ringShots.add(new WaitTask(400, TimeUnit.MILLISECONDS));
		}
		return new SequentialTaskBundle(
				ringShots.toArray(new Task[0])
		);
	}

	private SequentialTaskBundle shootPowerShots() {
		double SHOOTER_OFFSET = -4.5;

		return new SequentialTaskBundle (
				new TiltModeSelectTask(TiltModeSelectTask.Position.POWERSHOT, tilt),
				new StartShooterTask(shooter),
				followConstantTrajectory(() -> new Vector2d(-6, -8 + SHOOTER_OFFSET)),
				shootRings(1),
				followConstantTrajectory(() -> new Vector2d(-6, -15 + SHOOTER_OFFSET)),
				shootRings(1),
				followConstantTrajectory(() -> new Vector2d(-6, -22 + SHOOTER_OFFSET)),
				shootRings(1),
				new TiltModeSelectTask(TiltModeSelectTask.Position.BASE, tilt),
				new StopShooterTask(shooter)
		);
	}

	private FollowTrajectoryTask followConstantTrajectory(Supplier<Vector2d> vector) {
		return new FollowTrajectoryTask(
				drivetrain,
				() -> drivetrain.mecanum().trajectoryBuilder(drivetrain.mecanum().getPoseEstimate())
						.lineToConstantHeading(vector.get()).build()
		);
	}

	private FollowTrajectoryTask followTrajectory(Supplier<Pose2d> pose) {
		return new FollowTrajectoryTask(
				drivetrain,
				() -> drivetrain.mecanum().trajectoryBuilder(drivetrain.mecanum().getPoseEstimate())
						.lineToLinearHeading(pose.get()).build()
		);
	}

	private SequentialTaskBundle openGripper() {
		return new SequentialTaskBundle (
				new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.OPEN, gripper),
				new WaitTask(500, TimeUnit.MILLISECONDS)
		);
	}

	private SequentialTaskBundle closeGripper() {
		return new SequentialTaskBundle (
				new WobbleGripperControlTask(WobbleGripperControlTask.WobbleGripperState.CLOSE, gripper),
				new WaitTask(500, TimeUnit.MILLISECONDS)
		);
	}
}