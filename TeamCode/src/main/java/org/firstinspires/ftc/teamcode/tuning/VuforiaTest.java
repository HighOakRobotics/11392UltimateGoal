package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.RunTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.positioning.Position;
import org.firstinspires.ftc.teamcode.subsystem.positioning.VuforiaSensor;
import org.firstinspires.ftc.teamcode.subsystem.roadrunner.DashboardUtil;

import java.util.function.Supplier;

@TeleOp
public class VuforiaTest extends SequoiaOpMode {

    VuforiaSensor sensor = new VuforiaSensor();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        dashboard.startCameraStream(sensor.getVuforiaLocalizer(), 0);
        Supplier<Position> posSupplier = sensor.getPositionSupplier();
        gamepad1H.aToggleButton().whileOn(new InstantTask(() -> {
            TelemetryPacket packet = new TelemetryPacket();

            Position pos = posSupplier.get();

            double x = pos.getxPosition() != null ? pos.getxPosition() : -1000;
            double y = pos.getyPosition() != null ? pos.getyPosition() : -1000;
            double heading = pos.getHeading() != null ? pos.getHeading() : -1000;
            DashboardUtil.drawRobot(packet.fieldOverlay(), new Pose2d(x,y,heading));
            packet.put("x", x);
            packet.put("y", y);
            packet.put("r", heading);
            packet.put("time", pos.getTime());

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("r", heading);
            telemetry.addData("time", pos.getTime());

            dashboard.sendTelemetryPacket(packet);
        }));
    }
}