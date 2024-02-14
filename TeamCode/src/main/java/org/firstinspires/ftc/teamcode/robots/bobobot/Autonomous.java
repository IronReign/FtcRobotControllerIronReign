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
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.Constants;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.Toggle;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;

import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "BobotAutonomous")
public class Autonomous extends OpMode {
    private RunnerBot runnerBot;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;
    Toggle toggle;
    public static int autonIndex;
    public enum AutonState{
        DRIVE_FROM_START, TURN, DRIVE_THRU, DRIVE_TO_BACKDROP, INIT, PURPLE_PIXEL_DROP
    }
    Pose2d leftSpikeTape = new Pose2d(24, -24, 90);
    Pose2d middlespikeTape = new Pose2d(0,-24 ,90);
    Pose2d rightSpikeTape = new Pose2d(24, -30, 90);

    public static double GO_TO_X_RED;
    public static double GO_TO_Y_RED = -36;
    public AutonState autonState;
    private Action
    audienceRed, toRed, toBlue, toTape;

    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry,hardwareMap);
        toggle = new Toggle(gamepad1, gamepad2, runnerBot);
        autonState = AutonState.INIT;
        dashTelemetry.setMsTransmissionInterval(25);
        autonIndex = 0;
        runnerBot.driveTrain.setPose(Constants.Position.START_RIGHT_RED);
        runnerBot.start();
    }
    @Override
    public void init_loop(){
        toggle.gamepadUpdate();
        runnerBot.enableVision();
        toggle.autonSetup();
        telemetry.addData("Game Init Position \t", runnerBot.driveTrain.getGamePosition());
        telemetry.addData("Pose \t", "\n X \t"
                + runnerBot.driveTrain.pose.position.x + "\n Y \t"
                + runnerBot.driveTrain.pose.position.y + "\n Heading \t"
                + Math.toDegrees(runnerBot.driveTrain.pose.heading.log()));
        telemetry.addData("Spike Tape Index \t", runnerBot.driveTrain.getSpikeIndex());
    }
    @Override
    public void start(){


    }
    @Override
    public void loop(){
        purplePixelRed();
        /* switch(runnerBot.driveTrain.getGamePosition()){
            case START_LEFT_RED:
                break;
            case START_RIGHT_RED:
                break;
            case START_LEFT_BLUE:
                break;
            case START_RIGHT_BLUE:

        }*/
        telemetry.addData("Auton State \t", getAutonState());
        update();
        dashTelemetry.update();
    }

    public void goToRed() {
        toRed = new SequentialAction(
                runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                        .lineToY(GO_TO_Y_RED)
                        .build()
        );
    }

    public void goToBlue(){
        toBlue = new SequentialAction(
                runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                        .lineToY(-GO_TO_Y_RED)
                        .build()
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

    public void purplePixelRed(){
        switch (autonIndex){
            case 0:
                toTapeRed();
                autonIndex++;
                break;
            case 1:
                if (!toTape.run(new TelemetryPacket())) {
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

    public void purplePixelBlue(){
        switch (autonIndex){
            case 0:
                goToBlue();
                autonIndex++;
                break;
            case 1:
                if (!toBlue.run(new TelemetryPacket())) {
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
                goToRed();
                break;
            case 1:
                if(!toRed.run(new TelemetryPacket())) {
                    break;
                }
        }
    }

    public AutonState getAutonState() {
        return autonState;
    }

    public void toTapeRed(){
        goToRed();
        if(runnerBot.driveTrain.getSpikeIndex() == 1) {
            toTape = new SequentialAction(runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                    .splineToLinearHeading(leftSpikeTape, 0)
                    .build()
            );
        }
        else if(runnerBot.driveTrain.getSpikeIndex() == 2){
            toTape = toRed;

        }
        else if(runnerBot.driveTrain.getSpikeIndex() == 3) {
            toTape = new SequentialAction(runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                    .splineToLinearHeading(rightSpikeTape, 0)
                    .build()
            );
        }

    }



}
