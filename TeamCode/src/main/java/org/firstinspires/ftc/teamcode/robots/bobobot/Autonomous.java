package org.firstinspires.ftc.teamcode.robots.bobobot;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.RunnerBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "BobotAutonomous")
public class Autonomous extends OpMode {
    private RunnerBot runnerBot;
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
        build();
    }
    @Override
    public void start(){
        Actions.runBlocking(frontRed);
    }
    @Override
    public void loop(){

    }

    public void build(){
                frontRed = new SequentialAction(
                        runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                                .turn(Math.toRadians(-90))
                                .lineToX(64)
                        .build()
        );

                frontBlue = new SequentialAction(
                        runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                                .turn(Math.toRadians(90))
                                .lineToX(64)
                                .build()
                );

    }
}
