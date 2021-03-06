package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * New localizer from the drivetrain intended to provide the drivetrain with sensor fusion data,
 * instead of just odometry data.
 */
public class PositionLocalizer implements Localizer {
	Supplier<Position> positionSupplier;
	Consumer<Position> positionReset;
	Pose2d pose;

	public PositionLocalizer(Supplier<Position> positionSupplier, Consumer<Position> positionReset) {
		this.positionSupplier = positionSupplier;
		this.positionReset = positionReset;
	}

	public PositionLocalizer(Supplier<Position> positionSupplier) {
		this(positionSupplier, (Position pos) -> {
			throw new UnsupportedOperationException("This sensor adaptor does not support resets.");
		});
	}

	@NotNull
	@Override
	public Pose2d getPoseEstimate() {
		Position position = this.positionSupplier.get();
		return new Pose2d(new Vector2d(position.getX(), position.getY()), position.getHeading());
	}

	@Override
	public void setPoseEstimate(@NotNull Pose2d pose2d) {
		positionReset.accept(new Position(pose2d.getX(), pose2d.getY(), pose2d.getHeading()));
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity() {
		return null;
	}

	@Override
	public void update() { /*Do not update as this class just converts garbage*/ }
}
