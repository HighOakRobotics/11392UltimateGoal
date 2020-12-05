package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter2W;

public class IntakeTask extends Task {
    Intake intake;

    public IntakeTask(Shooter2W shooter) {
        this.running = true;
        this.intake = intake;
        addSubsystems(intake);
    }
    @Override
    public void init() {
        telemetry.addLine("Running Intake :)");
        intake.setIntakeVelocity(-1.0);
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop(boolean interrupted) {
        telemetry.addLine("Intake Stopped :(");
        intake.setIntakeVelocity(0.0);
    }

}
