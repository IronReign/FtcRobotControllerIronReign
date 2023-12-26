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
    DriveTrain mecanumAuto;
    FtcDashboard dashboard;
    Servo servoClaw;
    Servo clawWrist;
    public double TicksPerIn = 340.723404;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
//        AutoDrive.mecanumAuto(TicksPerIn * 3.000, 0, 0);
    }
}
