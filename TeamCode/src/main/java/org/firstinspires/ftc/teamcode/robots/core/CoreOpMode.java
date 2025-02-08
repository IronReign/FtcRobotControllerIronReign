package org.firstinspires.ftc.teamcode.robots.core;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.LinkedHashMap;
import java.util.Map;


import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "CORE")
public class CoreOpMode extends OpMode {
    Robot robot;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        dashboard = FtcDashboard.getInstance();
        robot.init();
        robot.calibrateStage = 0;
    }

    public void init_loop() {
        // calibrate by pressing gamepad1 guide (Logitech) button
        robot.initloopDrive();
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }
    @Override
    public void loop() {
        robot.update(new Canvas());

        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());

    }



    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        TelemetryPacket p = new TelemetryPacket();
        telemetry.addLine(telemetryName);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
            p.addLine(line);
        }
        telemetry.addLine();
        dashboard.sendTelemetryPacket(p);
    }
}


