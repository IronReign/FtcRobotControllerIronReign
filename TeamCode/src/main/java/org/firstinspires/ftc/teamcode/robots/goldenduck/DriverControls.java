package org.firstinspires.ftc.teamcode.robots.goldenduck;

import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Config ("GoldenDuckGameVariables")
@TeleOp(name="Golden Duck OpMode", group="Challenge")
public class DriverControls extends OpMode {
    private boolean calibrate = false;
    DriveTrain driveTrain;
    Servo servoClaw;
    Servo clawWrist;
//    Servo servoRailgun;
    FtcDashboard dashboard;
    private DcMotor
            arm = null;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");
//        servoRailgun = hardwareMap.get(Servo.class, "servoRailgun");
        driveTrain.motorInit();
        clawWrist = hardwareMap.get(Servo.class, "servoWrist");
        driveTrain = new DriveTrain(telemetry, hardwareMap);
        arm = this.hardwareMap.get
        (DcMotorEx.class, "arm");
//        telemetry.addData("arm position", arm.getCurrentPosition());
    }

    @Override
    public void loop() {
        telemetry.addData("servoWrist", clawWrist.getPosition());
        telemetry.addData("servoClaw", servoClaw.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
//        telemetry.addData("Railgun Shot"

        driveTrain.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.dpad_down) {
            calibrate = false;
        }
        if (gamepad1.dpad_up) {
            if (driveTrain.robotSpeed == 1)
                driveTrain.robotSpeed = .5;
            else
                driveTrain.robotSpeed = 1;
        }
        //open claw
        if (gamepad1.x) {
            arm.setPower(0.1);
            arm.setTargetPosition(-105);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            clawWrist.setPosition(servoNormalize(1047));
            //driving mode
        }
        if (gamepad1.y) {
            arm.setPower(0.2);
            arm.setTargetPosition(-87);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            clawWrist.setPosition(servoNormalize(1277));
            //to pick up pixel
        }
        if (gamepad1.b) {
            arm.setPower(0.3);
            arm.setTargetPosition(-1050);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            clawWrist.setPosition(0);
            // mid score backboard
        }
        if (gamepad1.a) {
            arm.setPower(0.3);
            arm.setTargetPosition(-1023);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            clawWrist.setPosition(0);
            // low score backboard
        }
        if (gamepad1.right_bumper) {
            servoClaw.setPosition(servoNormalize(1511));
            // claw open
        }
        if (gamepad1.left_bumper) {
            servoClaw.setPosition(servoNormalize(1821));
            //claw close
        }
//        if (gamepad1.dpad_left) {
//            servoRailgun.setPosition(servoNormalize(1821));
//        }
    }
}