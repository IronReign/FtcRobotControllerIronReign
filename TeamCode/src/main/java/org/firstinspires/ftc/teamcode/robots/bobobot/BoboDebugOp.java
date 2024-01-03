package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.IMU;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.Toggle;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

@TeleOp(name="BoboDebugOpMode", group="Challenge")
public class BoboDebugOp extends OpMode {
    public static DebugBot debugbot;
    IMU imu;
    Toggle toggle;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        debugbot = new DebugBot(dashTelemetry, hardwareMap);
        toggle = new Toggle(gamepad1, gamepad2);
        imu = new IMU(dashTelemetry, hardwareMap);
        dashTelemetry.setMsTransmissionInterval(25);
    }

    @Override
    public void loop() {
        toggle.gamepadUpdate();
        toggle.runTest();

        dashTelemetry.update();

    }

}
