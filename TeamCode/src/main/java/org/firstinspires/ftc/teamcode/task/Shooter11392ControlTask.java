package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class Shooter11392ControlTask extends Task {

    Shooter shooter;

    public Shooter11392ControlTask(Shooter shooter) {
        this.running = true;
        this.shooter = shooter;
        addSubsystems(shooter);
    }

    @Override
    public void init() {
        telemetry.log().add("started shooter");
        shooter.setFlywheelVelocity(1.0);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted) {
            telemetry.log().add("interrupted shooter");
        } else {
            telemetry.log().add("stopped shooter");
        }
        shooter.setFlywheelVelocity(0.0);
    }
}
