package org.firstinspires.ftc.teamcode.robots.spine;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;

@Disabled

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Spine", group = "Challenge")
public class SpineOp extends  OpMode {

        Spine robot;

        StickyGamepad stickyGamepad1;


        @Override
        public void init() {
            robot = new Spine(hardwareMap);
            robot.muscleTicks = 1500;
            stickyGamepad1 = new StickyGamepad(gamepad1);
        }

        @Override
        public void loop() {
            robot.update(new Canvas());
            robot.directDrive(gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.right_stick_y, -gamepad1.left_stick_x);
            telemetry.addData("muscle", robot.muscleTicks);
            telemetry.addData("rightPower", robot.rightPower);
            telemetry.addData("leftPower", robot.leftPower);
            updateTelemetry();
        }

        public void updateTelemetry() {

        }
}
