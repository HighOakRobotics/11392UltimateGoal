package org.firstinspires.ftc.teamcode.subsystem.positioning;

public class Position {
    private boolean hasX;
    private boolean hasY;
    private boolean hasHeading;

    private double xPosition; // inches
    private double yPosition; // inches
    private double heading; // radians

    public Position() {
        this(0.0, 0.0, 0.0);
    }

    public Position(Double x, Double y, Double heading) {

        hasX = x != null;
        hasY = y != null;
        hasHeading = heading != null;

        xPosition = x;
        yPosition = y;
        this.heading = heading;
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
