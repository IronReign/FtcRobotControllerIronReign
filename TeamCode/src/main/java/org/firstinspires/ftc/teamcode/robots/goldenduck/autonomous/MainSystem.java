package org.firstinspires.ftc.teamcode.robots.goldenduck.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.goldenduck.teleop.DriveTrain;

@Config("GoldenDuckGameVariables")
@Autonomous(name="Golden Duck Autonomous OpMode", group="Autonomous")
public class MainSystem extends LinearOpMode {
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


        waitForStart();
        motorFrontRight.setPower(-0.7);
        motorFrontLeft.setPower(0.7);
        motorBackRight.setPower(0.7);
        motorBackLeft.setPower(-0.7);
        sleep(30000);
    }
}
