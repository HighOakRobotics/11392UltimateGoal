package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Tilt;
import org.firstinspires.ftc.teamcode.subsystem.positioning.Position;

public class TiltModeSelectTask extends InstantTask {

    public TiltModeSelectTask(Positions position, Tilt tilt) {
        super(() -> {
            int target;
            switch (position) { // I feel like using enums incorrectly here.
                case BASE:
                    target = BASE;
                    break;
                case LOAD:
                    target = LOAD;
                    break;
                case SHOOT:
                    target = SHOOT;
                    break;
                case SHAKE:
                    target = SHAKE;
                    break;
                default:
                    target = LOAD;
                    break;
            }
            tilt.setTargetPosition(target);
        });
    }

    public enum Positions {
        BASE, LOAD, SHOOT, SHAKE
    }
    public static final int BASE = 0;
    public static final int LOAD = 140;
    public static final int SHOOT = 380;
    public static final int SHAKE = 500;

}