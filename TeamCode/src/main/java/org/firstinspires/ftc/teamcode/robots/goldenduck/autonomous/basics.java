package org.firstinspires.ftc.teamcode.robots.goldenduck.autonomous;

import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config("GoldenDuckGameVariables")
@Autonomous(name="Golden Duck Autonomous OpMode", group="Autonomous")
public class basics extends LinearOpMode {
    FtcDashboard dashboard;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackRight = null;
    private DcMotor arm = null;
    Servo servoClaw;
    Servo clawWrist;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("servoWrist", clawWrist.getPosition());
        telemetry.addData("servoClaw", servoClaw.getPosition());
        telemetry.addData("arm U/D position", arm.getCurrentPosition());
        motorFrontLeft = this.hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotorEx.class, "motorBackRight");
        this.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
//        motorFrontRight.setPower(0.7);
//        motorFrontLeft.setPower(0);
//        motorBackRight.setPower(0);
//        motorBackLeft.setPower(0.7);
//        sleep(1000);
//        motorFrontRight.setPower(-0.7);
//        motorFrontLeft.setPower(0.7);
//        motorBackRight.setPower(-0.7);
//        motorBackLeft.setPower(0.7);
//        sleep(500);
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackRight.setPower(0);
//        motorBackLeft.setPower(0);
//        arm.setPower(0.3);
//        arm.setTargetPosition(-1389);
//        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        clawWrist.setPosition(0);
//        sleep(500);
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackRight.setPower(0);
//        motorBackLeft.setPower(0);
//        arm.setPower(0);
//        servoClaw.setPosition(servoNormalize(900));
//        sleep(500);
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(0.7);
//        motorBackRight.setPower(0.7);
//        motorBackLeft.setPower(0);
//        sleep(30000);
    }
}