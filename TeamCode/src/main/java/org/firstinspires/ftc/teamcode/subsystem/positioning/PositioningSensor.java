package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Position;

import java.util.function.Supplier;

public class PositioningSensor extends Subsystem {
    Position position;

    public PositioningSensor() {
        this(new Position());
    }

    public PositioningSensor(Position initialPosition) {
        position = initialPosition;
    }

    public Supplier<Position> getPositionSupplier() {
        return () -> position;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) { }
    @Override
    public void runPeriodic() { }
    @Override
    public void initPeriodic() { }
    @Override
    public void stop() { }
}
