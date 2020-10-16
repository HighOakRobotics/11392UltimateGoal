package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

public class FusionSensor extends PositioningSensor {
    @Override
    public Supplier<Position> getPositionSupplier() {
        return null;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        priority = 10;
    }
}
