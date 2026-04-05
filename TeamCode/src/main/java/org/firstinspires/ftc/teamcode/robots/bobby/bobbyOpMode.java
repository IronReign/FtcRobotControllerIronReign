package org.firstinspires.ftc.teamcode.robots.bobby;


import com.acmerobotics.dashboard.canvas.Canvas;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.robots.deepthought.util.TelemetryProvider;


import java.sql.Driver;
import java.util.Map;


@TeleOp(name = "bobby", group = "game")
public class bobbyOpMode extends OpMode {
    private Robot robot;
    private StickyGamepad g1;
    private LaunchSequence shooterSequence;
    private DriverControls driverControls;


    @Override
    public void init() {
        robot = new Robot(hardwareMap, 0);
        g1 = new StickyGamepad(gamepad1);
        shooterSequence = new LaunchSequence();
        driverControls = new DriverControls();
    }


    @Override
    public void loop() {
        g1.update();
        driverControls.update(robot, gamepad1, g1, shooterSequence);
        robot.update(new Canvas());


        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
        for (TelemetryProvider subsystem : robot.subsystems) {
            handleTelemetry(subsystem.getTelemetry(true), subsystem.getTelemetryName());
        }
    }


    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        telemetry.addLine(telemetryName);
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
        }
        telemetry.addLine();
    }


    @Override
    public void stop() {
        robot.stop();
    }
}
