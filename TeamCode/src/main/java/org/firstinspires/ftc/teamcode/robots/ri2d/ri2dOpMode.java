package org.firstinspires.ftc.teamcode.robots.ri2d;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.Map;

@TeleOp(name = "0 - ri2d")
public class ri2dOpMode extends OpMode {


    public static Robot robot;
    DcMotorEx leftHook, rightHook;
    public static DriverControls dc;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        dc = new DriverControls(gamepad1);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void loop() {
        dc.joystickDrive();
        TelemetryPacket p = new TelemetryPacket();
        handleTelemetry(robot.swerve.getTelemetry(true), robot.swerve.getTelemetryName(), p);
        robot.update(new Canvas());
    }
    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet) {
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }
}
