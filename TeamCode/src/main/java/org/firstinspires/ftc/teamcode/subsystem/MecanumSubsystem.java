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

    public DriveTrainMecanum mecanum() {
        return mecanum;
    }
}
