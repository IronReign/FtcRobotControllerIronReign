package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Swerve", group = "Challenge")
public class SwerveOp extends OpMode {

CrapSwerve robot;

StickyGamepad stickyGamepad1;

boolean dampenRotation = false;

    @Override
    public void init() {
        robot = new CrapSwerve(hardwareMap);

        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        robot.update(new Canvas());

        if(stickyGamepad1.a) {
            dampenRotation = !dampenRotation;
        }

        if(Math.abs(gamepad1.left_stick_y) > .05) {
            robot.goPower = gamepad1.left_stick_y;
            telemetry.addData("goPower: ", "stick (%.2f), power (%.2f)", gamepad1.left_stick_y, robot.goPower);
        }
        else robot.goPower = 0;
        if(Math.abs(gamepad1.right_stick_x) > .05){
            robot.yawPower = dampenRotation ? gamepad1.right_stick_x/4 :gamepad1.right_stick_x;
        }
        else robot.yawPower = 0;
    }
}
