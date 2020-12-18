package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;

import org.firstinspires.ftc.teamcode.subsystem.WobbleGripper;

public class WobbleGripperControlTask extends InstantTask {
    public enum WobbleGripperState {
        OPEN, CLOSE
    }

    public WobbleGripperControlTask(WobbleGripperState state, WobbleGripper gripper) {
        super(() -> {
            switch (state) {
                case OPEN:
                    gripper.open();
                    break;
                case CLOSE:
                    gripper.close();
                    break;
            }
        });
    }
}
