package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends Subsystem {
        private double intakeVelocity;
        private DcMotorEx IntakeMotor;
        public double getIntakeVelocity() {
        return intakeVelocity;
    }

    public void setIntakeVelocity(double intakeVelocity) {
        this.intakeVelocity = intakeVelocity;
    }
        public void initialize(HardwareMap hardwareMap){
            IntakeMotor.setPower(0);
        }

    @Override
    public void start() {
        IntakeMotor.setVelocity(0);
    }

    @Override
    public void runPeriodic() {
        IntakeMotor.setVelocity(intakeVelocity);
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void stop() {
        IntakeMotor.setVelocity(0);
    }
}
