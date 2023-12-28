package org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Robot;
import org.firstinspires.ftc.teamcode.robots.bobobot.IMU;

@TeleOp(name="BoboDebugOpMode", group="Challenge")
public class BoboDebugOp extends OpMode {
    Robot bobot;
    IMU imu;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        bobot = new Robot(dashTelemetry, hardwareMap);
        imu = new IMU(dashTelemetry, hardwareMap);
        dashTelemetry.setMsTransmissionInterval(25);
    }

    @Override
    public void loop() {
        bobot.opDrive.runTest(gamepad1.a);
        bobot.opDrive.telemetryOutput();
        dashTelemetry.update();

    }
}
