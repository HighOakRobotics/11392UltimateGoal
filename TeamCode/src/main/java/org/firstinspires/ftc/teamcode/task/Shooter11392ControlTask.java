package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Shooter11392;

public class Shooter11392ControlTask extends Task {

    Shooter11392 shooter;

    public Shooter11392ControlTask(Shooter11392 shooter) {
        this.running = true;
        this.shooter = shooter;
        addSubsystems(shooter);
    }

    @Override
    public void init() {
        shooter.setDesiredFlywheelVelocity(1000.0);
    }

    @Override
    public void loop() {
        telemetry.addData("shooter speed", shooter.getFlywheelVelocity());
    }

    @Override
    public void stop(boolean interrupted) {
        shooter.setDesiredFlywheelVelocity(0.0);
    }
}
