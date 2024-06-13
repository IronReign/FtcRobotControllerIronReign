package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Swerve", group = "Challenge")
public class SwerveOp extends OpMode {

CrapSwerve robot;

    @Override
    public void init() {
        robot = new CrapSwerve(hardwareMap);
    }

    @Override
    public void loop() {
        robot.update(new Canvas());

        if(Math.abs(gamepad1.left_stick_y) > .05) {
            robot.goPower = gamepad1.left_stick_y;
            telemetry.addData("goPower: ", "stick (%.2f), power (%.2f)", gamepad1.left_stick_y, robot.goPower);
        }
        else robot.goPower = 0;
        if(Math.abs(gamepad1.right_stick_x) > .05){
            robot.yawPosition = Range.clip(robot.yawPosition + (int)(gamepad1.right_stick_x * 20),(1500-750),(1500+750));
        }
    }
}
