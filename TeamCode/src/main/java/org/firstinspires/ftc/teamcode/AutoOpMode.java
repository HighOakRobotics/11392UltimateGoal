package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.ftc11392.sequoia.task.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.legacy.HolyExt;
import org.firstinspires.ftc.teamcode.subsystem.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Loader;
import org.firstinspires.ftc.teamcode.subsystem.RingDetector;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;
import org.firstinspires.ftc.teamcode.subsystem.WobbleGripper;
import org.firstinspires.ftc.teamcode.task.TiltModeSelectTask;

@Autonomous(name = "AutoOpMode 11392", group = "11392", preselectTeleOp = "DriveOpMode 11392")
//@Disabled
public class AutoOpMode extends LinearOpMode {
    private HolyExt holy;
    private Lift lift;
    private Tilt tilt;
    private WobbleGripper gripper;
    private Shooter shooter;
    private Loader loader;
    private RingDetector ringDetector;

    // SampleMecanumDrive drive;

    private double TURN_KMI = 0.08;
    private double TURN_KP = 0.06;
    private double TURN_KI = 0.0018;
    private double TURN_KD = 2.5;
    private double TURN_KF = 0.0;
    private int nRings = 0;

    @Override
    public void runOpMode() {

        resetScheduler();
        Scheduler.getInstance().init(telemetry);

        holy = new HolyExt(hardwareMap, telemetry,1,1,1,true, this);
        holy.init();
        holy.resetEncoder();
        holy.initRT(0.8);
        holy.turnOffBrakes();
        holy.setTPR(DriveConstants.TICKS_PER_REV);
        holy.setMaxI(TURN_KMI);
        holy.setPIDTurnCoefficients(new PIDFCoefficients(TURN_KP, TURN_KI, TURN_KD, TURN_KF));

        shooter = new Shooter();
        loader = new Loader();
        gripper = new WobbleGripper();
        tilt = new Tilt();

        Scheduler.getInstance().initSubsystems(hardwareMap);
        waitForStart();
        Scheduler.getInstance().startSubsystems();

        gripper.close();
        shooter.setDesiredFlywheelVelocity(2000);
        shooter.runPeriodic();
        tilt.setTargetPosition(357);
        tilt.runPeriodic();
        sleep(5000);
        loader.setLoaderState(Loader.LoaderState.OPEN);
        loader.runPeriodic();
        sleep(2000);
        loader.setLoaderState(Loader.LoaderState.CLOSED);
        loader.runPeriodic();
        sleep(5000);
        loader.setLoaderState(Loader.LoaderState.OPEN);
        loader.runPeriodic();
        sleep(2000);
        loader.setLoaderState(Loader.LoaderState.CLOSED);
        loader.runPeriodic();
        sleep(5000);
        loader.setLoaderState(Loader.LoaderState.OPEN);
        loader.runPeriodic();
        sleep(2000);
        loader.setLoaderState(Loader.LoaderState.CLOSED);
        loader.runPeriodic();
        sleep(2000);

        holy.ourStrafeByInchRT(5, 0.15, this);
        holy.driveByInchSynchRT(78,0.15, this);
        sleep(1000);
        gripper.open();

        stop();

    }

    private void resetScheduler() {
        Scheduler.getInstance().cancelAll();
        Scheduler.getInstance().clearBehaviors();
        Scheduler.getInstance().clearSubsystems();
    }

}