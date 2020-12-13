package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends Subsystem{

    private DcMotorEx lift;
    private double liftPower;
    public int startPosition = 0;
    public int targetPosition = 0;
    public double getLiftPower(){
        return liftPower;
    }

    public void setLiftPower(double liftPower){
        this.liftPower = liftPower;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setTargetPosition(startPosition);
    }

    @Override
    public void start() {
        lift.setTargetPosition(targetPosition);
    }

    @Override
    public void runPeriodic() {
        //set the gamepad thing here i guess
        //replace the 0 i mean with whatever controls you want for the gamepad
        targetPosition = 0;
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void stop() {

    }
}
