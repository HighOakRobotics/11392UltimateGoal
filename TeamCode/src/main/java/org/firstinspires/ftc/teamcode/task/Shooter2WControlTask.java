package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Shooter2W;

public class Shooter2WControlTask extends Task {

    Shooter2W shooter;

    public Shooter2WControlTask(Shooter2W shooter) {
        this.running = true;
        this.shooter = shooter;
        addSubsystems(shooter);
    }

    @Override
    public void init() {
        telemetry.log().add("started shooter");
        shooter.setFrontFlywheelVelocity(1.0);
        shooter.setBackFlywheelVelocity(1.0);
    }

    @Override
    public void loop() { }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted) {
            telemetry.log().add("interrupted shooter");
        } else {
            telemetry.log().add("stopped shooter");
        }
        shooter.setFrontFlywheelVelocity(0.0);
        shooter.setBackFlywheelVelocity(0.0);
    }
}
