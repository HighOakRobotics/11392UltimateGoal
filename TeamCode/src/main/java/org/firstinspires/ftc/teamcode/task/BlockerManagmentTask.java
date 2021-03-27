package org.firstinspires.ftc.teamcode.task;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystem.Blockers;
import org.firstinspires.ftc.teamcode.subsystem.positioning.Position;

import java.util.function.Supplier;

public class BlockerManagmentTask extends Task {

	Supplier<Position> positionSupplier;
	Blockers blockers;

	public BlockerManagmentTask(Blockers blockers, Supplier<Position> positionSupplier) {
		this.blockers = blockers;
		this.positionSupplier = positionSupplier;
	}

	@Override
	public void init() {
		blockers.close();
	}

	@Override
	public void loop() {
		Position pos = positionSupplier.get();
		double x = pos.getX();
		double y = pos.getY();
		// -72 < x < 72, -72 < y < 24
		// -48 < x < 48, -48 < y < 0
		if ( x >= -48 && x <= 48 && y >= -48 && y <= 0)
			blockers.open();
		else blockers.close();
	}

	@Override
	public void stop(boolean interrupted) {	}
}
