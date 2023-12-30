package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.IMU;

@TeleOp(name="BoboDebugOpMode", group="Challenge")
public class BoboDebugOp extends OpMode {
    RunnerBot runnerBot;
    IMU imu;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry, hardwareMap);
        imu = new IMU(dashTelemetry, hardwareMap);
        dashTelemetry.setMsTransmissionInterval(25);
    }

    @Override
    public void loop() {

        dashTelemetry.update();

    }
}
