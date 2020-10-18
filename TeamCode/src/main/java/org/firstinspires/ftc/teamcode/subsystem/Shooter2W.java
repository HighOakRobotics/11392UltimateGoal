package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter2W extends Subsystem {

    private final double INIT_TRACK_ANGLE = 0.0;
    private final double INIT_SHOOTER_PITCH = 0.0;
    private DcMotorEx frontFlywheel;
    private DcMotorEx backFlywheel;
    private Servo loader;
    private Servo track;
    private Servo pivot;
    private double frontFlywheelVelocity;
    private double backFlywheelVelocity;
    private double trackAngle;
    private double shooterPitch;

    public double getFrontFlywheelVelocity() {
        return frontFlywheelVelocity;
    }
    public double getBackFlywheelVelocity() {
        return backFlywheelVelocity;
    }

    public void setFrontFlywheelVelocity(double frontFlywheelVelocity) {
        this.frontFlywheelVelocity = frontFlywheelVelocity;
    }

    public void setBackFlywheelVelocity(double backFlywheelVelocity) {
        this.backFlywheelVelocity = backFlywheelVelocity;
    }

    public double getTrackAngle() {
        return trackAngle;
    }

    public void setTrackAngle(double trackAngle) {
        this.trackAngle = trackAngle;
    }

    public double getShooterPitch() {
        return shooterPitch;
    }

    public void setShooterPitch(double shooterPitch) {
        this.shooterPitch = shooterPitch;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {

        frontFlywheel = hardwareMap.get(DcMotorEx.class, "frontFlywheel");
        backFlywheel = hardwareMap.get(DcMotorEx.class, "backFlywheel");

        // TODO Grab hardware devices
        //track.setPosition(INIT_TRACK_ANGLE);
        //pivot.setPosition(INIT_SHOOTER_PITCH);

        frontFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //flywheel.setMotorType();
        frontFlywheel.setPower(0); // In encoder mode it runs at a fraction of maximum velocity.
        backFlywheel.setPower(0);
    }

    @Override
    public void runPeriodic() {
        frontFlywheel.setPower(frontFlywheelVelocity);
        backFlywheel.setPower(backFlywheelVelocity);
        //track.setPosition(trackAngle);
        //pivot.setPosition(shooterPitch);
    }

    @Override
    public void initPeriodic() { }

    @Override
    public void stop() {
        frontFlywheel.setPower(0);
        backFlywheel.setPower(0);
    }
}
