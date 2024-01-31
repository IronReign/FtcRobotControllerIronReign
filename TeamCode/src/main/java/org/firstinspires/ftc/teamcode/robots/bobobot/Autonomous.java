package org.firstinspires.ftc.teamcode.robots.bobobot;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;

import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "BobotAutonomous")
public class Autonomous extends OpMode {
    private RunnerBot runnerBot;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;

    private Action
    test, audienceRed, backdropRed,
    audienceBlue, backdropBlue;
    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry,hardwareMap);
        //runnerBot.driveTrain.setPose(Constants.Position.START_RIGHT_RED);
        runnerBot.driveTrain.setPose(Constants.Position.START_LEFT_RED);
        dashTelemetry.setMsTransmissionInterval(25);
        build();
    }
    @Override
    public void start(){
    }
    @Override
    public void loop(){
        if(!audienceRed.run(new TelemetryPacket())) {
            //telemetry done
        }
        update();
        dashTelemetry.update();
    }

    public void build(){
        //todo on: START_LEFT_RED
                audienceRed = new SequentialAction(
                        runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                                .splineTo(new Vector2d(-25, 60), -180)
                        .build()
        );

        //todo on: START_RIGHT_RED
                backdropRed = new SequentialAction(
                        runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                                .turn(Math.toRadians(-90))
                                .lineToX(-25)
                                .build()
                  );
        //todo on: START_LEFT_BLUE
                audienceBlue = new SequentialAction(

                );
        //todo on: START_RIGHT_BLUE
                backdropBlue = new SequentialAction(

                );
    }
    private void update(){
        TelemetryPacket packet = new TelemetryPacket();

        for(TelemetryProvider telemetryProvider: runnerBot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(false), telemetryProvider.getTelemetryName(), packet);
        runnerBot.update(packet.fieldOverlay());
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    public void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet){
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);
        packet.addLine("");

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }

}
