package org.firstinspires.ftc.teamcode.subsystem.positioning;

public class Position {
    double xPosition; // inches
    double yPosition; // inches
    double heading; // radians

    public Position() {
        this(0, 0, 0);
    }

    public Position(double x, double y, double heading) {
        xPosition = x;
        yPosition = y;
        this.heading = heading;
    }

    public double getxPosition() {
        return xPosition;
    }

    public void setxPosition(double xPosition) {
        this.xPosition = xPosition;
    }

    public double getyPosition() {
        return yPosition;
    }

    public void setyPosition(double yPosition) {
        this.yPosition = yPosition;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }
}
