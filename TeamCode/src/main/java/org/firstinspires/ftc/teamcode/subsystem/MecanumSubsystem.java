package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystem.DriveConstants.getMotorVelocityF;

public class MecanumSubsystem extends Subsystem {
    DriveTrainMecanum mecanum;
    @Override
    public void initialize(HardwareMap hardwareMap) {
        mecanum = new DriveTrainMecanum(hardwareMap);
        mecanum.setMotorPowers(0,0,0,0);
        mecanum.update();
    }

    @Override
    public void runPeriodic() {
        mecanum.update();
    }

    @Override
    public void initPeriodic() {
        mecanum.update();
    }

    @Override
    public void stop() {
        mecanum.setMotorPowers(0,0,0,0);
        mecanum.update();
    }

    public void setDriveDST() {
        mecanum.setDriveDST();
    }

    public void setDriveDST(DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn) {
        mecanum.setDriveDST(drive, strafe, turn);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return mecanum.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return mecanum.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return mecanum.trajectoryBuilder(startPose, startHeading);
    }

    public void turn(double angle) {
        mecanum.turn(angle);
    }

    public void followTrajectory(Trajectory trajectory) {
        mecanum.followTrajectory(trajectory);
    }

    public Pose2d getLastError() {
        return mecanum.getLastError();
    }

    public boolean isBusy() {
        return mecanum.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        mecanum.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        mecanum.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        return mecanum.getPIDCoefficients(runMode);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        mecanum.setPIDCoefficients(runMode, coefficients);
    }

    public List<Double> getWheelPositions() {
        return mecanum.getWheelPositions();
    }

    public List<Double> getWheelVelocities() {
        return mecanum.getWheelVelocities();
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        mecanum.setMotorPowers(v, v1, v2, v3);
    }

    public double getRawExternalHeading() {
        return mecanum.getRawExternalHeading();
    }
}
