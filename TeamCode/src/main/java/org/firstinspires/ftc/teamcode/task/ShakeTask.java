package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;

public class ShakeTask extends Task {

    Tilt tilt;
    Shooter shooter;
    Clock clock;

    public ShakeTask(Tilt tilt, Shooter shooter) {
        this.tilt = tilt;
        this.shooter = shooter;
    }
    @Override
    public void init() {
        shooter.start();
        tilt.se
        clock = new Clock();

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(boolean interrupted) {

    }
}
