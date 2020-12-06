package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Transfer;

public class TransferTask extends Task {

    Transfer transfer;

    public TransferTask(Transfer transfer) {
        this.transfer = transfer;
    }

    @Override
    public void init() {
        transfer.runWheels();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(boolean interrupted) {
        transfer.stopWheels();
    }
}
