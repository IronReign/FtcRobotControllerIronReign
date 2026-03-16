package org.firstinspires.ftc.teamcode.robots.bobby;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "BOBBY VISION TUNER", group = "Tuning")
public class VisionTuner extends OpMode {

    Robot robot;
    FtcDashboard dashboard;

    static final double CAMERA_CENTER_X = 320; // adjust if resolution differs

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1,0);
        robot.init();   // pipeline + camera still initialized here

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        sendDashboardTelemetry();
        drawOverlay();
        //
    }

    private void sendDashboardTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Ball X", robot.pipeline.ballX);
        packet.put("Ball Area", robot.pipeline.ballArea);
        packet.put("Ball Seen", robot.pipeline.ballX != -1);

        dashboard.sendTelemetryPacket(packet);
    }

    private void drawOverlay() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // Camera center indicator
        canvas.setStroke("#00FF00");
        canvas.strokeLine(0, -10, 0, 10);

        // Detected object visualization
        if (robot.pipeline.ballX != -1) {
            double ballRelativeX =
                    (robot.pipeline.ballX - CAMERA_CENTER_X) / 10.0;

            double radius = Math.max(2, robot.pipeline.ballArea / 1000.0);

            canvas.setStroke("#FF0000");
            canvas.strokeCircle(ballRelativeX, 0, radius);
        }

        dashboard.sendTelemetryPacket(packet);
    }
}
