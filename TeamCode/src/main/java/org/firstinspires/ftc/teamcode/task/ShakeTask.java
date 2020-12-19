package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;

public class ShakeTask extends Task {

    Tilt tilt;
    Shooter shooter;
    Clock clock;

    boolean haltTrigger = false;

    public ShakeTask(Tilt tilt, Shooter shooter) {
        this.tilt = tilt;
        this.shooter = shooter;
    }
    @Override
    public void init() {
        tilt.setTargetPosition(TiltModeSelectTask.Position.SHAKE.pos());
        clock = new Clock();
        clock.startTiming();
        this.running = true;
        shooter.setDesiredFlywheelVelocity(2000);
        haltTrigger = true;
    }

    @Override
    public void loop() {
        if (haltTrigger)
        if (clock.getMillis() >= 2000) {
            running = false;
            stop(false);
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (haltTrigger) {
            shooter.setDesiredFlywheelVelocity(0);
            tilt.setTargetPosition(TiltModeSelectTask.Position.LOAD.pos());
        }
        haltTrigger = false;
    }
}
