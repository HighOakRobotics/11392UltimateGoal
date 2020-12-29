package org.firstinspires.ftc.teamcode.tuning;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.RunTask;

import org.firstinspires.ftc.teamcode.subsystem.positioning.Position;
import org.firstinspires.ftc.teamcode.subsystem.positioning.VuforiaSensor;

import java.util.function.Supplier;

public class VuforiaTest extends SequoiaOpMode {

    VuforiaSensor sensor = new VuforiaSensor();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        Supplier<Position> posSupplier = sensor.getPositionSupplier();
        runLoopTrigger.rising(new RunTask(() -> {
            Position pos = posSupplier.get();
            telemetry.addData("x", pos.getxPosition());
            telemetry.addData("y", pos.getyPosition());
            telemetry.addData("r", pos.getHeading());
            telemetry.addData("time", pos.getTime());
        }, sensor));
    }
}
