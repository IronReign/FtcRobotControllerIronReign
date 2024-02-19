package org.firstinspires.ftc.teamcode.robots.bobobot;


import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@Config(value = "BoadBunnerAuto")
public class Autonomous extends OpMode {
    private RunnerBot runnerBot;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;
    Toggle toggle;
    public static int autonIndex;
    public int visionProviderIndex;
    public static int stageIndex;
    public enum AutonState{
        DRIVE_FROM_START, TURN, DRIVE_THRU, DRIVE_TO_BACKDROP, INIT, PURPLE_PIXEL_DROP
    }
    Pose2d leftSpikeTape = new Pose2d(24, -24, 90);
    Pose2d middlespikeTape = new Pose2d(0,-24 ,90);
    Pose2d rightSpikeTape = new Pose2d(24, -30, 90);

    public static double GO_TO_X_RED;
    public static double GO_TO_Y_RED = -36;
    public static double GO_TO_Y_BLUE = -GO_TO_Y_RED;
    public static double turnToSpeed = .4;

    public static int turnTo;
    public AutonState autonState;
    private Action
    toRed, toBlue, toTape, strafeToBackRed, strafeToBackBlue, toBack;

    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry,hardwareMap);
        toggle = new Toggle(gamepad1, gamepad2, runnerBot);
        autonState = AutonState.INIT;
        dashTelemetry.setMsTransmissionInterval(25);
        autonIndex = 0;
        //runnerBot.driveTrain.setPose(Constants.Position.START_RIGHT_RED);
        runnerBot.start();
    }
    @Override
    public void init_loop(){
        toggle.gamepadUpdate();
        runnerBot.enableVision();
        updateIndexOffsets();
        toggle.autonSetup();
        telemetry.addData("Game Init Position \t", runnerBot.driveTrain.getGamePosition());
        telemetry.addData("Game Alliance \t", runnerBot.driveTrain.getAlliance());
        telemetry.addData("Pose \t", "\n X \t"
                + runnerBot.driveTrain.pose.position.x + "\n Y \t"
                + runnerBot.driveTrain.pose.position.y + "\n Heading \t"
                + Math.toDegrees(runnerBot.driveTrain.pose.heading.log()));
        telemetry.addData("visionProviderIndex", visionProviderIndex);
        telemetry.addData("Turn To Degrees? \t", turnTo);
        telemetry.addData("Target Index \t", targetIndex);
        telemetry.addData("Red Alliance? \t", Constants.Alliance.RED.getMod());
        telemetry.addData("Blue Alliance \t", Constants.Alliance.BLUE.getMod());

    }
    @Override
    public void start(){
        runnerBot.visionProvider.shutdownVision();

    }
    @Override
    public void loop(){
        Auton(runnerBot.driveTrain.getGamePosition());
        telemetry.addData("Auton State \t", getAutonState());
        telemetry.addData("Auton Index \t", autonIndex);
        telemetry.addData("Stage Index", stageIndex);
        update();
        dashTelemetry.update();
    }

    public void goToRed() {
        toRed = new SequentialAction(
                runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                        .lineToY(GO_TO_Y_RED)
                        //.turn(Math.toRadians(turnTo))
                        .build()
        );
    }

    public void goToBlue(){
        toBlue = new SequentialAction(
                runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                        .lineToY(GO_TO_Y_BLUE)
                        //.turn(Math.toRadians(turnTo))
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
                if(targetIndex != 2 && runnerBot.driveTrain.turnUntilDegreesIMU(turnTo,turnToSpeed) ){
                    autonIndex++;
                }
                else if (targetIndex == 2){
                    autonIndex++;
                }

                break;
            case 3:
                runnerBot.intake.armWristOut();
                autonState = AutonState.PURPLE_PIXEL_DROP;
                futureTimer = futureTime(2);
                autonIndex++;
                break;
            case 4:
                if(isPast(futureTimer)) {
                    runnerBot.intake.closeClaw();
                }
                break;
        }
    }

    public void purplePixelBlue(){
        switch (autonIndex){
            case 0:
                toTapeBlue();
                autonIndex++;
                break;
            case 1:
                if (!toTape.run(new TelemetryPacket())) {
                    autonState = AutonState.DRIVE_FROM_START;
                    autonIndex++;
                }
                break;
            case 2:
                if( targetIndex!= 2 && runnerBot.driveTrain.turnUntilDegreesIMU(turnTo,turnToSpeed)){
                    autonIndex++;
                }
                else if (targetIndex == 2){
                    autonIndex++;
                }
            case 3:
                runnerBot.intake.armWristOut();
                autonState = AutonState.PURPLE_PIXEL_DROP;
                futureTimer = futureTime(2);
                autonIndex++;
                break;
            case 4:
                if(isPast(futureTimer)) {
                    runnerBot.intake.closeClaw();
                }
                break;
        }
    }


    public AutonState getAutonState() {
        return autonState;
    }

    public void toTapeRed(){
        goToRed();
        toTape = toRed;
    }

    public void toTapeBlue(){
        goToBlue();
        toTape = toBlue;
    }

    public void Auton(Constants.Position position){
         switch(position){
            case START_LEFT_RED:
                purplePixelRed();
                break;
            case START_RIGHT_RED:
                purplePixelRed();
                break;
            case START_LEFT_BLUE:
                purplePixelBlue();
                break;
            case START_RIGHT_BLUE:
                purplePixelBlue();
        }
    }

    public void runGameRed(){
        switch (stageIndex){
            case 0:
                purplePixelRed();
                runFullRed();
                stageIndex++;
                break;
            case 1:
                if(!toBack.run(new TelemetryPacket()))
                    break;
        }
    }

    public void runGameBlue(){
        switch (stageIndex){
            case 0:
                purplePixelBlue();
                runFullBlue();
                stageIndex++;
                break;
            case 1:
                if(!toBack.run(new TelemetryPacket()))
                    break;
        }
    }
    public void backBlue(){
        strafeToBackBlue = new SequentialAction(runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                .lineToX(43)
                .build()
        );
    }

    public void backRed(){
        strafeToBackRed = new SequentialAction(runnerBot.driveTrain.actionBuilder(runnerBot.driveTrain.pose)
                .lineToX(43)
                .build()
        );
    }

    public void runFullRed(){
        backRed();
        toBack = strafeToBackRed;
    }

    public void runFullBlue(){
        backBlue();
        toBack = strafeToBackBlue;
    }
    public static int targetIndex;
    public void updateIndexOffsets(){
        visionProviderIndex = runnerBot.visionProvider.getMostFrequentPosition().getIndex();
        targetIndex = visionProviderIndex + 1;

        if(runnerBot.driveTrain.getGamePosition().equals(Constants.Position.START_RIGHT_RED)){
            if(targetIndex == 3){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = 90;
            }
            if(targetIndex == 2){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = 30;
            }
            if(targetIndex == 1){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = -90;
            }
        }

        if(runnerBot.driveTrain.getGamePosition().equals(Constants.Position.START_LEFT_RED)){
            if(targetIndex == 3){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = 90;
            }
            if(targetIndex == 2){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = 30;
            }
            if(targetIndex == 1){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = -90;
            }
        }

        if(runnerBot.driveTrain.getGamePosition().equals(Constants.Position.START_RIGHT_BLUE)){
            if(targetIndex == 3){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = 90;
            }
            if(targetIndex == 2){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = 30;
            }
            if(targetIndex == 1){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = -90;
            }

        if(runnerBot.driveTrain.getGamePosition().equals((Constants.Position.START_LEFT_BLUE))){
            if(targetIndex == 3){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = 90;
                }
            if(targetIndex == 2){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = 30;
                }
            if(targetIndex == 1){
                //runnerBot.visionProvider.shutdownVision();
                turnTo = -90;
                }
            }
        }
    }


}
