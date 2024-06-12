package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Swerve", group = "Challenge")
public class SwerveOp extends OpMode {


    CrapSwerve robot = new CrapSwerve(hardwareMap);
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        robot.update(new Canvas());

        if(gamepad1.left_stick_y > .05) {
            robot.goPosition += (gamepad1.left_stick_y * 20);
        }
        if(gamepad1.left_stick_x > .05){
            robot.yawPosition += (gamepad1.left_stick_x * 20);
        }
    }
}
