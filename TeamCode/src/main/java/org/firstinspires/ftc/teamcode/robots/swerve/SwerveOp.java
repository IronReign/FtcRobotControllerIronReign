package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Swerve", group = "Challenge")
public class SwerveOp extends OpMode {

CrapSwerve robot;

StickyGamepad stickyGamepad1;


    @Override
    public void init() {
        robot = new CrapSwerve(hardwareMap);
        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        robot.update(new Canvas());
        robot.simplySwerve(Math.abs(gamepad1.right_stick_x), gamepad1.left_stick_x, gamepad1.left_stick_y);
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("raw encoder position", robot.yawEncoder.getCurrentPosition());
        telemetry.addData("real yaw", robot.realYaw);
        telemetry.addData("target yaw", robot.targetYaw);
        telemetry.addData("yaw error", robot.yawController.getError());
        telemetry.addData("yaw power", robot.yawPower);

//      **** DIRECT DRIVE ONLY ****
//        telemetry.addData("dampen?: ", dampenRotation);
//        telemetry.addData("goPower: ", "stick (%.2f), power (%.2f)", gamepad1.left_stick_y, robot.goPower);
    }
}
