package org.firstinspires.ftc.teamcode.robots.ri2d2;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Map;

@TeleOp(name = "TwoDays", group = "twoDays")
public class NewBotTeleOp extends OpMode {
    private Robot robot;
    private DriverControls driverControls;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.stop();
        driverControls = new DriverControls(gamepad1, robot);
    }

    @Override
    public void loop() {
        driverControls.update();
        robot.update(new Canvas());
        for (Map.Entry<String, Object> entry : robot.catapult.getTelemetry(true).entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
//        for (Map.Entry<String, Object> entry : robot.flywheel.getTelemetry(false).entrySet()) {
//            telemetry.addData(entry.getKey(), entry.getValue());
//        }
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }
}
