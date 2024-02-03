package org.firstinspires.ftc.teamcode.robots.bobobot;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;

import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "BobotAutonomous")
public class Autonomous extends OpMode {
    private RunnerBot runnerBot;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;
    public static int autonIndex;
    public enum AutonState{
        DRIVE_FROM_START, TURN, DRIVE_THRU, DRIVE_TO_BACKDROP, INIT, PURPLE_
    }
    public AutonState autonState;
    private Action
    audienceRed, backdropRed,
    audienceBlue, backdropBlue, through;
    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry,hardwareMap);
        //runnerBot.driveTrain.setPose(Constants.Position.START_LEFT_RED);
        autonState = AutonState.INIT;
        dashTelemetry.setMsTransmissionInterval(25);
        autonIndex = 0;

    }
    @Override
    public void init_loop(){
        runnerBot.enableVision();
    }
    @Override
    public void start(){
        runnerBot.driveTrain.pose = Constants.Position.START_RIGHT_RED.getPose();
        runnerBot.start();
    }
    @Override
    public void loop(){
        //runAud();
        runBack();
        update();
        dashTelemetry.update();
    }

    public void buildAudRed() {
        //todo on: START_LEFT_RED
        audienceRed = new SequentialAction(
                runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                        .lineToY( 0)
                        .build()
        );
    }
        //todo on: START_RIGHT_RED
    public void buildBackRed() {
        backdropRed = new SequentialAction(
                runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                        .lineToX(42)
                        .build()
        );
    }
    public void getAudBlue() {
        //todo on: START_LEFT_BLUE
        audienceBlue = new SequentialAction(
        );
    }
    public void buildBackBlue(){
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
    public void run(){
        switch (autonIndex){
            case 0:
                buildAudRed();
                autonIndex++;
                break;
            case 1:
                if (!audienceRed.run(new TelemetryPacket()))
                autonState = AutonState.DRIVE_FROM_START;
                autonIndex++;
                break;
            case 2:
                runnerBot.intake.openClaw();

                autonIndex++;
                break;
            case 3:

                break;
        }
    }
    public void runBack() {
        switch(autonIndex) {
            case 0:
                if(runnerBot.driveTrain.turnUntilDegreesIMU(0,.8)){
                runnerBot.driveTrain.pose = new Pose2d(new Vector2d(runnerBot.driveTrain.pose.position.x, runnerBot.driveTrain.pose.position.y),Math.toRadians(runnerBot.driveTrain.imuAngle));
                autonIndex++;
                }
                buildBackRed();
                break;
            case 1:
                if(!backdropRed.run(new TelemetryPacket())) {
                    break;
                }
        }
    }
    public void throughBuild(){
        through = new SequentialAction((runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                .lineToX(-40)
                .build()
        ));
    }
}
