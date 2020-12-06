package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends Subsystem {
        private double intakePower;
        private DcMotorEx intake;
        public double getIntakePower() {
        return intakePower;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void initialize(HardwareMap hardwareMap){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intake.setPower(0);
        }

    @Override
    public void start() {
        intake.setPower(0);
    }

    @Override
    public void runPeriodic() {
        intake.setPower(intakePower);
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void stop() {
        intake.setPower(0);
    }
}
