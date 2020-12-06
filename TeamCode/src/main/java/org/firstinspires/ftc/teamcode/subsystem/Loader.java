package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Loader extends Subsystem {
    Servo loader;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        loader = hardwareMap.get(Servo.class, "loader");
        loader.setPosition(1);
    }

    public void close() {
        loader.setPosition(1);
    }

    public void open() {
        loader.setPosition(0);
    }

    @Override
    public void start() { }
    @Override
    public void runPeriodic() { }
    @Override
    public void initPeriodic() { }
    @Override
    public void stop() { }
}
