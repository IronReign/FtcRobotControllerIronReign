package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Swerve", group = "Challenge")
public class SwerveOp extends OpMode {

CrapSwerve robot;

StickyGamepad stickyGamepad1;

public static boolean dampenRotation = false;

    @Override
    public void init() {
        robot = new CrapSwerve(hardwareMap);
        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        robot.update(new Canvas());
        robot.simplySwerve(gamepad1.left_stick_x, gamepad1.left_stick_y);
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("raw encoder position", robot.yawEncoder.getCurrentPosition());
//        telemetry.addData("dampen?: ", dampenRotation); -- use for direct drive
        telemetry.addData("real yaw", robot.realYaw);
        telemetry.addData("target yaw", robot.targetYaw);
        telemetry.addData("yaw error", robot.yawController.getError());
        telemetry.addData("yaw power", robot.yawPower);
    }

    public void directDrive() {
        if(gamepad1.a) {
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
