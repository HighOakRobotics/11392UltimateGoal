package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class ShooterControlTask extends Task {

    Shooter shooter;

    public ShooterControlTask(Shooter shooter) {
        this.running = true;
        this.shooter = shooter;
        addSubsystems(shooter);
    }

    @Override
    public void init() {
        shooter.setDesiredFlywheelVelocity(2000.0);
    }

    @Override
    public void loop() { }

    @Override
    public void stop(boolean interrupted) {
        shooter.setDesiredFlywheelVelocity(0.0);
    }
}
