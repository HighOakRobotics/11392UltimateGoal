package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer extends Subsystem {

    CRServo secondWheels;
    CRServo thirdWheels;

    boolean state;

    public void runWheels() {
        state = true;
    }

    public void stopWheels() {
        state = false;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        secondWheels = hardwareMap.get(CRServo.class, "secondWheels");
        thirdWheels = hardwareMap.get(CRServo.class, "thirdWheels");

        secondWheels.setDirection(DcMotorSimple.Direction.FORWARD);
        thirdWheels.setDirection(DcMotorSimple.Direction.FORWARD);

        secondWheels.setPower(0);
        thirdWheels.setPower(0);

        state = false;
    }

    @Override
    public void initPeriodic() { }

    @Override
    public void start() { }

    @Override
    public void runPeriodic() {
        if (state) {
            secondWheels.setPower(1);
            thirdWheels.setPower(1);
        } else {
            secondWheels.setPower(0);
            thirdWheels.setPower(0);
        }
    }

    @Override
    public void stop() {
        secondWheels.setPower(0);
        thirdWheels.setPower(0);
    }
}
