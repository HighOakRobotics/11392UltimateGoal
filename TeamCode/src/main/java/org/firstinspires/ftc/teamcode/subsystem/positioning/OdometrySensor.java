package org.firstinspires.ftc.teamcode.subsystem.positioning;

import java.util.function.Supplier;

public class OdometrySensor extends PositioningSensor {

    public StandardTrackingWheelLocalizer odometry;

    @Override
    public Supplier<Position> getPositionSupplier() {
        return null;
    }
}
