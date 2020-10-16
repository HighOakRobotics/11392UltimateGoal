package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.function.Supplier;

/**
 * New localizer from the drivetrain intended to provide the drivetrain with sensor fusion data,
 * instead of just odometry data.
 */
public class PositionLocalizer implements Localizer {
    Supplier<Position> fusionSupplier;
    Pose2d pose;

    public PositionLocalizer(Supplier<Position> fusionSupplier) {
        this.fusionSupplier = fusionSupplier;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        Position position = fusionSupplier.get();
        return new Pose2d(new Vector2d(position.getxPosition(), position.getyPosition()), position.getHeading());
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        // TODO check if this actually makes sense for road runner
        throw new UnsupportedOperationException("FusionLocalizer should not have its pose set.");
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() { /*Do not update as this just converts garbage*/ }
}
