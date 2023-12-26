package org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.RunnerBot;

@Autonomous(name = "BoboRunner")
public class BoboRunnerOp extends OpMode {
    RunnerBot runnerBot;
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
        dashTelemetry.setMsTransmissionInterval(25);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void loop(){

    }


    public void frontRed(){

    }
}
