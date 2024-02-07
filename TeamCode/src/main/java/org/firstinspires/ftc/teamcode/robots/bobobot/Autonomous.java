package org.firstinspires.ftc.teamcode.robots.bobobot;


import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.Intake;
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
        DRIVE_FROM_START, TURN, DRIVE_THRU, DRIVE_TO_BACKDROP, INIT, PURPLE_PIXEL_DROP
    }

    public static double GO_TO_X_AUD;
    public static double GO_TO_Y_AUD = -36;
    public static double GO_TO_X_BACK;
    public static double GO_TO_Y_BACK = GO_TO_Y_AUD;
    public AutonState autonState;
    private Action
    audienceRed, backdropRed,
    audienceBlue, backdropBlue, through;

    public static Constants.Alliance alliance = Constants.Alliance.RED;
    public static boolean backdrop = false;
    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry,hardwareMap);
        runnerBot.driveTrain.setPose(Constants.Position.START_LEFT_RED);
        //alliance = Constants.Alliance.BLUE;
        autonState = AutonState.INIT;
        dashTelemetry.setMsTransmissionInterval(25);
        autonIndex = 0;
        runnerBot.start();
    }
    @Override
    public void init_loop(){
        runnerBot.enableVision();
    }
    @Override
    public void start(){
//        runnerBot.driveTrain.pose = Constants.Position.START_RIGHT_RED.getPose();
//        if(!alliance.getMod()){
//            if(!backdrop){
//                runnerBot.driveTrain.setPose(Constants.Position.START_LEFT_BLUE);
//            }
//            else if (backdrop){
//                runnerBot.driveTrain.setPose(Constants.Position.START_RIGHT_BLUE);
//            }
//        }
//        else if (alliance.getMod()){
//            if(!backdrop){
//                runnerBot.driveTrain.setPose(Constants.Position.START_LEFT_RED);
//            }
//            else if (backdrop){
//                runnerBot.driveTrain.setPose(Constants.Position.START_RIGHT_RED);
//            }
//        }

    }
    @Override
    public void loop(){
        purplePixel();
        update();
        dashTelemetry.update();
    }

    public void buildAudRed() {
        //todo on: START_LEFT_RED
        audienceRed = new SequentialAction(
                runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                        .lineToY( -36)
                        .build()
        );
    }
        //todo on: START_RIGHT_RED
    public void buildBackRed() {
        backdropRed = new SequentialAction(
                runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                        .lineToY(-36)
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
    public static long futureTimer;

    public void purplePixel(){
        switch (autonIndex){
            case 0:
                buildBackRed();
                autonIndex++;
                break;
            case 1:
                if (!backdropRed.run(new TelemetryPacket())) {
                    autonState = AutonState.DRIVE_FROM_START;
                    autonIndex++;
                }
                break;
            case 2:
                runnerBot.intake.armWristOut();
                autonState = AutonState.PURPLE_PIXEL_DROP;
                futureTimer = futureTime(2);
                autonIndex++;
                break;
            case 3:
                if(isPast(futureTimer)) {
                    runnerBot.intake.closeClaw();
                }
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
 through = new SequentialAction(
         runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                 .splineTo(new Vector2d(0,24),0)
                 .build()
 );
}

}
