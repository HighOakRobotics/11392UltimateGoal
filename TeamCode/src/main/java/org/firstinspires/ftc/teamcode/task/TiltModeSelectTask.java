package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.InstantTask;
import org.firstinspires.ftc.teamcode.subsystem.Tilt;

public class TiltModeSelectTask extends InstantTask {

    public TiltModeSelectTask(Position position, Tilt tilt) {
        super(() -> tilt.setTargetPosition(position.pos()));
    }

    public enum Position {
        BASE(0),
        LOAD(140),
        SHOOT(380),
        SHAKE(500);

        Position(int pos) {
            position = pos;
        }
        private final int position;
        public int pos() {
            return position;
        }
    }
}
