package org.firstinspires.ftc.teamcode.robots.goldenduck.teleop;

import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class DriverControlsOld {

//this is just the remains of previous gamepad processing for reference
    //todo delete this file when no longer needed
//
//        if (gamepad1.x) {
//            arm.setPower(0.1);
//            arm.setTargetPosition(15);
//            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            clawWrist.setPosition(servoNormalize(-500));
//            //driving mode
//        }
//
//        if (gamepad1.y) {
//            arm.setPower(0.2);
//            //arm.setTargetPosition(105);
//            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            clawWrist.setPosition(servoNormalize(-1000));
//            //to pick up pixel
//        }
//
//        if (gamepad1.b) {
//            arm.setPower(0.3);
//            //arm.setTargetPosition(1389);
//            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            clawWrist.setPosition(0);
//            // mid score backboard
//        }
//
//        if (gamepad1.a) {
//            arm.setPower(0.3);
//            //arm.setTargetPosition(1615);
//            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            clawWrist.setPosition(0);
////      low score backboard
//        }
//
//        if (gamepad1.right_bumper) {
//            servoClaw.setPosition(servoNormalize(850));
////      claw open
//        }
//
//        if (gamepad1.left_bumper) {
//            servoClaw.setPosition(servoNormalize(1300));
////      claw close
//        }
//
//        if (gamepad1.dpad_down) {
//            servoRailgun.setPosition(servoNormalize(1821));
//        }

//        if (gamepad1.dpad_right) {
//            arm2.setPower(xyz);
//            arm2.getCurrentPosition() + xyz;
//            arm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        }
//
//         if (gamepad1.dpad_right) {
//            arm2.setPower(xyz);
//            arm2.getCurrentPosition() - xyz;
//            arm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//       }

}