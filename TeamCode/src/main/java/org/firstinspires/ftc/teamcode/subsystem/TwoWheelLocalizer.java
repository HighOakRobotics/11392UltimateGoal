package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.roadrunner.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {

	public static double TICKS_PER_REV = 8192; // 2048 quadrature cycles
	public static double WHEEL_RADIUS = 0.74; // in
	public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

	private Encoder centerEncoder;
	private Encoder rightEncoder;

	private Supplier<Double> headingSupplier;

	public TwoWheelLocalizer(HardwareMap hardwareMap, Supplier<Double> headingSupplier) {
		super(Arrays.asList(
				new Pose2d(-2.473, 0.087, -1*Math.PI / 2), //horizontal (center)
				new Pose2d(-2.915, -2.773, 0) //vertical (right)
		));

		centerEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
		rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));

		// Encoder.setDirection(Encoder.Direction.REVERSE)

		this.headingSupplier = headingSupplier;
	}

	@Override
	public double getHeading() {
		return headingSupplier.get();
	}

	@Override
	public @NotNull List<Double> getWheelPositions() {
		return Arrays.asList(
				encoderTicksToInches(centerEncoder.getCurrentPosition()),
				encoderTicksToInches(rightEncoder.getCurrentPosition())
		);
	}

	public static double encoderTicksToInches(double ticks) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}
}
