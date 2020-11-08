package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.RunTask;
import com.ftc11392.sequoia.task.Task;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.task.ShooterControlTask;

import java.util.function.BooleanSupplier;

@TeleOp(name="Tele 16457", group="16457")
@Disabled
public class TeleOp16457 extends SequoiaOpMode {
    Shooter shooter = new Shooter();
    //OdometrySensor odometry = new OdometrySensor();
    //MecanumSubsystem drivetrain = new MecanumSubsystem(odometry.getPositionSupplier());

    @Override
    public void initTriggers() { }

    @Override
    public void runTriggers() {
        gamepad1H.aToggleButton().risingWithCancel(//new ParallelTaskBundle(
                new ShooterControlTask(shooter)//,
                //new RunTask(() -> {
                //	telemetry.addData("shooter", "running");
                //}, null)
                //)
        );
        //gamepad1H.sticksButton(0.05).onPressWithCancel(new GamepadDriveTask(drivetrain, gamepad1));
    }
}
