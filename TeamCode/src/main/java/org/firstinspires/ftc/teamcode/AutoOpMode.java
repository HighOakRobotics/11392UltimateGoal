package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.ftc11392.sequoia.task.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.legacy.HolyExt;
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


    private int nRings = 0;

    @Override
    public void runOpMode() {

        resetScheduler();

        holy = new HolyExt(hardwareMap, telemetry, this);
        holy.init();
        holy.runWithEncoders();
        holy.initRT(0.8);
        /*drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        */
        lift = new Lift();
        lift.initialize(hardwareMap);

        tilt = new Tilt();
        gripper = new WobbleGripper();
        shooter = new Shooter();
        loader = new Loader();
        ringDetector = new RingDetector();

        Scheduler.getInstance().initSubsystems(hardwareMap);
        Scheduler.getInstance().startSubsystems();

        tilt.setTargetPosition(TiltModeSelectTask.Position.SHOOT.pos());
        tilt.runPeriodic();

        while (!isStarted()) {
            nRings = ringDetector.numOfRings();
            telemetry.addData("rings: ", "%4d", nRings);
            telemetry.update();
        }
        ringDetector.stop();

        waitForStart();

        //deliver wobble
        gripper.close();
        sleep(500);
        lift.modifyTarget(100);
        lift.runPeriodic();

        //to shoot power poles position
        shooter.setDesiredFlywheelVelocity(2000); //get ready to shoot power pole
        holy.driveByInchSynchRT(55, 0.35, this);
        holy.ourStrafeByInchRT(-6, 0.35, this);
        //shoot right one
        sleep(1000);
        loader.setLoaderState(Loader.LoaderState.OPEN); //power pole 1
        sleep(1000);
        loader.setLoaderState(Loader.LoaderState.CLOSED);
        //holy.strafeByInchSynch(8, 0.3, this);
        sleep(1000);
        loader.setLoaderState(Loader.LoaderState.OPEN); //power pole 2
        sleep(1000);
        loader.setLoaderState(Loader.LoaderState.CLOSED);
        //holy.strafeByInchSynch(8, 0.3, this);
        sleep(1000);
        loader.setLoaderState(Loader.LoaderState.OPEN); //power pole 3
        sleep(1000);
        loader.setLoaderState(Loader.LoaderState.CLOSED);
        shooter.stop();

        if (nRings == 4) { //C
            holy.driveByInchSynchRT(68, 0.5, this);
            sleep(200);
            holy.ourStrafeByInchRT(-32, 0.5, this);
            lift.modifyTarget(-100);
            gripper.open();
            holy.ourStrafeByInchRT(10,0.2, this);
            holy.driveByInchSynchRT(-28, 1.0, this);
            sleep(30000);
        } else if (nRings == 1) { //B
            holy.driveByInchSynchRT(47, .3, this);
            sleep(200);
            //holy.strafeByInchSynch(-5, 0.5, this);
            lift.modifyTarget(-100);
            gripper.open();
            //holy.strafeByInchSynch(8, 0.4, this);
            // holy.driveByInchSynch(-85, .4, this, telemetry);
            //  holy.strafeByInchSynch(-26, 0.6, this);
            //   holy.strafeByInchSynch(-5.5, 0.1, this);
            //  sleep(400);
            //   arm.close();
            //  sleep(700);
            //   lift.moveByInch(3, .6);
            //  holy.strafeByInchSynch(29, 0.4, this);
            //    holy.driveByInchSynch(85, .6, this, telemetry);
            //     holy.strafeByInchSynch(-16, 0.4, this);
            //    lift.bottom();
            //      sleep(200);
            //     arm.open();
            sleep(300);
            holy.ourStrafeByInchRT(8,0.5, this);
            holy.driveByInchSynchRT(-24, 0.3, this);
            sleep(30000);
        } else { //default A
            //holy.driveByInchSynch(a1.getY()-shoot3.getY(), .6, this);
            //holy.strafeByInchSynch(-(a1.getX()-shoot3.getX()), 0.5, this); //straft left
            holy.driveByInchSynchRT(24, .3, this);
            sleep(200);
            holy.ourStrafeByInchRT(-32, 0.5, this);
            lift.modifyTarget(-100);
            gripper.open();
            //holy.strafeByInchSynch(28, 0.4, this);
            ////  holy.driveByInchSynch(-66, .4, this, telemetry);
            // holy.strafeByInchSynch(-17, 0.5, this);
            // holy.strafeByInchSynch(-5, 0.1, this);
            // sleep(400);
            //  arm.close();
            //  sleep(700);
            //  holy.driveByInchSynch(65, .6, this, telemetry);
            //  holy.strafeByInchSynch(-12, 0.5, this);
            //  sleep(200);
            //  arm.open();
            sleep(300);
            holy.ourStrafeByInchRT(8,0.5, this);
            holy.driveByInchSynchRT(-4, 0.3, this);
            sleep(30000);
                /*holy.strafeByInchSynch(a1.getX()-shoot3.getX(), 0.5, this); //straft right
                holy.driveByInchSynch(shoot3.getY()-wobble.getY(), -.6, this);
                arm.close();
                lift.moveByInch(2, .5);
                holy.driveByInchSynch(a2.getY()-wobble.getY(), .6, this);//back
                holy.strafeByInchSynch(-5, 0.5, this); //straft left
                 */
        }

        //telemetry.addData("left hand: ","at %4.1f", arm.leftHand.getPosition());
        //telemetry.addData("right hand: ","at %4.1f", arm.rightHand.getPosition());
        //telemetry.addData("lift: ","at %4.1f", lift.liftMotor.getCurrentPosition());
        telemetry.update();


    }

    private void resetScheduler() {
        Scheduler.getInstance().cancelAll();
        Scheduler.getInstance().clearBehaviors();
        Scheduler.getInstance().clearSubsystems();
    }

}