package org.firstinspires.ftc.teamcode.robots.cipher;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.cipher.Robot;

import java.util.Map;

@TeleOp
@Config(value = "CIPHER")
public class cipherOpMode extends OpMode {
    Robot robot;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        dashboard = FtcDashboard.getInstance();
        robot.init();
    }

    public void init_loop() {
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }

    @Override
    public void start()
    {

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


