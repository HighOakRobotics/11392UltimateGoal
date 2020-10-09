package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainMecanumSubsystem extends Subsystem {
    DriveTrainMecanum mecanum;
    @Override
    public void initialize(HardwareMap hardwareMap) {
        mecanum = new DriveTrainMecanum(hardwareMap);
    }

    @Override
    public void runPeriodic() {

    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void stop() {

    }
}
