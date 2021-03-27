package org.firstinspires.ftc.teamcode.subsystem;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lights extends Subsystem {

    RevBlinkinLedDriver blinkinLedDriver;

    RevBlinkinLedDriver.BlinkinPattern patternPlay = RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE;
    RevBlinkinLedDriver.BlinkinPattern patternEnd = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
    RevBlinkinLedDriver.BlinkinPattern finalTen = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;

    public void endGame() {
        blinkinLedDriver.setPattern(patternEnd);
    }

    public void finalTen() {
        blinkinLedDriver.setPattern(finalTen);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(patternPlay);
    }

    @Override
    public void initPeriodic() { }

    @Override
    public void start() { }

    @Override
    public void runPeriodic() { }

    @Override
    public void stop() { }
}
