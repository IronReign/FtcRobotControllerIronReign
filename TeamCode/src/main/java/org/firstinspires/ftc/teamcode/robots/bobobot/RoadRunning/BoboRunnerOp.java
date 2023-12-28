package org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.bobobot.IMU;

@TeleOp(name = "BoboRunner")
public class BoboRunnerOp extends OpMode {
    RunnerBot runnerBot;
    IMU imu;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;

    private Action
            frontRed, backRed,
            frontBlue, backBlue;
    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry,hardwareMap);
        imu = new IMU(dashTelemetry, hardwareMap);
        dashTelemetry.setMsTransmissionInterval(25);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void loop(){
        runnerBot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        imu.telemetryOutput();
        runnerBot.driveTrain.getTelemetryName();
        runnerBot.driveTrain.getTelemetry(false);
        dashTelemetry.update();
    }


    public void frontRed(){

    }
}
