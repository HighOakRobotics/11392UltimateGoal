package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystem.Loader;

public class LoaderPushTask extends Task {

    Loader loader;
    Clock clock;

    public LoaderPushTask(Loader loader) {
        this.loader = loader;
        running = true;
        clock = new Clock();
    }

    @Override
    public void init() {
        loader.open();
        clock.startTiming();
    }

    @Override
    public void loop() {
        if (clock.getMillis() >= 500) {
            running = false;
        }
        telemetry.addData("loader time",clock.getMillis());
    }

    @Override
    public void stop(boolean interrupted) {
        telemetry.log().add("loader stopped by scheduler");
        loader.close();
    }
}
