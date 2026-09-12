package org.firstinspires.ftc.teamcode.robots.newBot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Map;

@TeleOp(name = "NewBotTeleOp", group = "newBot")
public class NewBotTeleOp extends OpMode {
    private Robot robot;
    private DriverControls driverControls;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.stop();
        driverControls = new DriverControls(gamepad1, robot);

        telemetry.addLine("Left stick: drive/strafe. Right stick: turn.");
        telemetry.addLine("A: flywheel on/off. RB: intake. LB: reverse intake.");
        telemetry.addData("Flywheel mode", robot.flywheel.getTelemetry(false).get("Mode"));
    }

    @Override
    public void loop() {
        driverControls.update();
        robot.update(new Canvas());
        for (Map.Entry<String, Object> entry : robot.flywheel.getTelemetry(false).entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }
}
