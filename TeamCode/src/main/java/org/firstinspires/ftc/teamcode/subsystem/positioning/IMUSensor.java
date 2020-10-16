package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.function.Supplier;

public class IMUSensor extends PositioningSensor {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    Position position;

    private void update() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity = imu.getGravity();

        // TODO Ensure that imu is operating correctly by checking status and calibration

        position = new Position(null,null, (double) angles.firstAngle);
    }

    @Override
    public Supplier<Position> getPositionSupplier() {
        return () -> position;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        super.initialize(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.startAccelerationIntegration(new org.firstinspires.ftc.robotcore.external.navigation.Position(), new Velocity(), 1000);
    }

    @Override
    public void initPeriodic() {
        update();
    }

    @Override
    public void runPeriodic() {
        update();
    }
}
