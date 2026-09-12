package org.firstinspires.ftc.teamcode.robots.newBot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "NewBotTeleOp", group = "newBot")
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
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }
}
