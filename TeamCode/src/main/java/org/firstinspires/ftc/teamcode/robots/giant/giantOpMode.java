package org.firstinspires.ftc.teamcode.robots.giant;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.LinkedHashMap;
import java.util.Map;
@TeleOp(name="giant mode", group="game")
public class giantOpMode extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        robot.init();
    }

    @Override
    public void loop() {
        robot.update(new Canvas());
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        telemetry.addLine(telemetryName);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
        }

        telemetry.addLine();
    }
}
