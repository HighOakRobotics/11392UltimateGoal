package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Tilt extends Subsystem {

    DcMotorEx tilt;
    int tiltPosition;
    final int MAX_POS = 0;
    final int MIN_POS = -350;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        tilt = hardwareMap.get(DcMotorEx.class, "tilt");
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltPosition = tilt.getCurrentPosition();
        tilt.setTargetPosition(tiltPosition);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt.setPower(0);
    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {

    }

    @Override
    public void initPeriodic() {
        telemetry.addData("tiltPos", tilt.getCurrentPosition());
    }

    @Override
    public void stop() {

    }
}
