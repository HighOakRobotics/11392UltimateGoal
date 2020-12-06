package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.PIDFSubsystem;
import com.ftc11392.sequoia.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

public class Tilt extends PIDFSubsystem {

    public static final int BASE_POSITION = 0;
    public static final int LOADING_POSITION = 0;
    public static final int DEFAULT_SHOOTING_POSITION = 0;

    DcMotorEx tilt;
    Supplier<Integer> feedback;
    int initPosition;

    public Tilt() {
        super(new PIDFController(1,0,1));
        disable();
    }

    @Override
    protected void useOutput(double output) {
        tilt.setPower(output);
    }

    @Override
    protected double getFeedback() {
        return feedback.get();
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        tilt = hardwareMap.get(DcMotorEx.class, "tilt");
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedback = tilt::getCurrentPosition;
        initPosition = feedback.get();
    }

    @Override
    public void start() {
        enable();
    }



    @Override
    public void stop() {
        disable();
    }
}
