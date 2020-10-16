package org.firstinspires.ftc.teamcode.subsystem.positioning;

import java.util.concurrent.TimeUnit;

public class Position {
    private final double time;

    private final boolean hasX;
    private final boolean hasY;
    private final boolean hasHeading;

    private final double xPosition; // inches
    private final double yPosition; // inches
    private final double heading; // radians

    public Position() {
        this(0.0, 0.0, 0.0);
    }

    public Position(Double x, Double y, Double heading) {
        time = TimeUnit.SECONDS.convert(System.nanoTime(), TimeUnit.NANOSECONDS);

        hasX = x != null;
        hasY = y != null;
        hasHeading = heading != null;

        xPosition = x;
        yPosition = y;
        this.heading = heading;
    }

    public double getTime() {
        return time;
    }

    public double getxPosition() {
        return xPosition;
    }

    public double getyPosition() {
        return yPosition;
    }

    public double getHeading() {
        return heading;
    }

    public boolean hasX() {
        return hasX;
    }

    public boolean hasY() {
        return hasY;
    }

    public boolean hasHeading() {
        return hasHeading;
    }
}
