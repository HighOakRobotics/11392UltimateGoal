package org.firstinspires.ftc.teamcode.tuning;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.RunTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.positioning.Position;
import org.firstinspires.ftc.teamcode.subsystem.positioning.VuforiaSensor;

import java.util.function.Supplier;

@TeleOp
public class VuforiaTest extends SequoiaOpMode {

    VuforiaSensor sensor = new VuforiaSensor();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        Supplier<Position> posSupplier = sensor.getPositionSupplier();
        gamepad1H.aButton().whilePressed(new InstantTask(() -> {
            Position pos = posSupplier.get();
            if (pos != null) {
                telemetry.addData("x", pos.getxPosition());
                telemetry.addData("y", pos.getyPosition());
                telemetry.addData("r", pos.getHeading());
                telemetry.addData("time", pos.getTime());
            }
        }));
    }
}