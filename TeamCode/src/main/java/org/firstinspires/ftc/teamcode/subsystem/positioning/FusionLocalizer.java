package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * New localizer from the drivetrain intended to provide the drivetrain with sensor fusion data,
 * instead of just odometry data.
 *
 * This architecture could have a lot of latency so it is subject to change.
 */
public class FusionLocalizer implements Localizer {
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return null;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        // TODO get data from suppliers
    }
}
