package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.FIELD_INCHES_PER_GRID;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.P2D;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Skyhook;
import org.firstinspires.ftc.teamcode.robots.csbot.util.CSPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.LinkedHashMap;
import java.util.Map;


@Config(value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {

    public VisionProvider visionProvider;
    private Robot robot;
    private HardwareMap hardwareMap;

    public enum AutonState{
        INIT,
        BACK_UP,
        STRAFE,
        SCORE_GROUND,
        TRAVEL_BACKSTAGE,
        TRAVEL_BACKDROP,
        SCORE_BACKDROP
    }

    public AutonState autonState = AutonState.INIT;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("autonState", autonState);
        telemetryMap.put("targetIndex", targetIndex);
        telemetryMap.put("indexStrafeOffset", indexStrafeOffset);
        telemetryMap.put("indexHeadingOffset", indexHeadingOffset);
        telemetryMap.put("visionProvider index", visionProviderIndex);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTONOMOUS";
    }

    // autonomous routines

    public static int targetIndex = 1;
    public static int visionProviderIndex;
    int allianceDirection = -1;
    double yellowOffset = 0;

    double standardHeading = 180;
    private Action
            redLeftStageOne, redLeftStageTwo,
            blueLeftStageOne, blueLeftStageTwo,
            blueRightStageOne, blueRightStageTwo,
            redRightStageOne, redRightStageTwo,
            travelBackstage, travelBackdrop
    ;

    private Action stageOneToRun, stageTwoToRun;

    // misc. routines
    public StateMachine backAndForth, square, turn;

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        this.visionProvider = robot.visionProviderBack;
    }

    Pose2d blueRightStageOnePosition;
    Pose2d blueRightStageTwoPosition;

    Pose2d blueLeftStageOnePosition;
    Pose2d blueLeftStageTwoPosition;

    Pose2d redRightStageOnePosition;
    Pose2d redRightStageTwoPosition;

    Pose2d redLeftStageOnePosition;
    Pose2d redLeftStageTwoPosition;

    Pose2d purpleEndPosition;
    Pose2d backdropApproachPosition;

    Pose2d backdropFinalPosition;


    public static int INWARD_SCORING_ANGLE = -45;
    public static int MIDDLE_SCORING_ANGLE = -50;
    public static int OUTWARD_SCORING_ANGLE = 30;
    public static double indexHeadingOffset = 0;
    public static double indexStrafeOffset = 0;

    //BASED ON BLUE_RIGHT_START
    public static double STAGE_ONE_Y_COORDINATE = .5;

    public static double STAGE_TWO_X_COORDINATE = -2.1;
    public static double STAGE_TWO_HEADING = 40;
    public static double STAGE_ONE_HEADING = 90;
    public static double BACKSTAGE_X_POSITION_OFFSET = 2.5;

    public void updateIndexOffsets() {
        visionProviderIndex = robot.visionProviderBack.getMostFrequentPosition().getIndex();
        targetIndex = visionProviderIndex + 1;

        if(startingPosition.equals(Constants.Position.START_RIGHT_BLUE)) {
            if (targetIndex == 1) {
                STAGE_ONE_Y_COORDINATE = 0.65;
                STAGE_TWO_X_COORDINATE = -1.65;
                STAGE_TWO_HEADING = 90 + INWARD_SCORING_ANGLE;
            }

            if (targetIndex == 2) {
                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_HEADING = 90 + MIDDLE_SCORING_ANGLE;
                STAGE_TWO_X_COORDINATE = -2.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_X_COORDINATE = -1.55;
                STAGE_TWO_HEADING = 90 + OUTWARD_SCORING_ANGLE;
            }
        }

        if(startingPosition.equals(Constants.Position.START_LEFT_BLUE)) {
            if (targetIndex == 1) {
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_X_COORDINATE = 1.55;
                STAGE_TWO_HEADING = 90 - OUTWARD_SCORING_ANGLE;
            }

            if (targetIndex == 2) {
                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_HEADING = 90 - MIDDLE_SCORING_ANGLE;
                STAGE_TWO_X_COORDINATE = 2.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_Y_COORDINATE = 0.65;
                STAGE_TWO_X_COORDINATE = 1.65;
                STAGE_TWO_HEADING = 90 - INWARD_SCORING_ANGLE;
            }
        }

        if(startingPosition.equals(Constants.Position.START_LEFT_RED)) {
            if (targetIndex == 1) {
                STAGE_ONE_Y_COORDINATE = -0.65;
                STAGE_TWO_X_COORDINATE = -1.65;
                STAGE_TWO_HEADING = -(90 + OUTWARD_SCORING_ANGLE);
            }

            if (targetIndex == 2) {
                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_HEADING = -(90 + MIDDLE_SCORING_ANGLE);
                STAGE_TWO_X_COORDINATE = -2.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_X_COORDINATE = -1.55;
                STAGE_TWO_HEADING = -(90 + INWARD_SCORING_ANGLE);
            }
        }
        if(startingPosition.equals(Constants.Position.START_RIGHT_RED)) {
            if (targetIndex == 1) {
                STAGE_ONE_Y_COORDINATE = -0.65;
                STAGE_TWO_X_COORDINATE = 1.65;
                STAGE_TWO_HEADING = -(90 - INWARD_SCORING_ANGLE);
            }

            if (targetIndex == 2) {
                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_HEADING = -(90 - MIDDLE_SCORING_ANGLE);
                STAGE_TWO_X_COORDINATE = 2.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_X_COORDINATE = 1.55;
                STAGE_TWO_HEADING = (90 - OUTWARD_SCORING_ANGLE);
            }
        }


    }

    public void build() {
        autonIndex = 0;
        futureTimer = 0;

        Pose2d pose = startingPosition.getPose();

        blueRightStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, STAGE_ONE_HEADING);
        blueRightStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, STAGE_TWO_HEADING);

        blueLeftStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, STAGE_ONE_HEADING);
        blueLeftStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE + BACKSTAGE_X_POSITION_OFFSET - indexStrafeOffset, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, STAGE_TWO_HEADING);

        redLeftStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, -STAGE_ONE_HEADING);
        redLeftStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE + indexStrafeOffset, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, -STAGE_TWO_HEADING);

        redRightStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, -STAGE_ONE_HEADING);
        redRightStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, -STAGE_TWO_HEADING);

        purpleEndPosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, allianceDirection * .5, standardHeading);
        backdropApproachPosition = P2D(2.0, allianceDirection * .5, standardHeading);
        backdropFinalPosition = P2D(2.0, allianceDirection * 1.5 + yellowOffset/FIELD_INCHES_PER_GRID, standardHeading);

        //
        redLeftStageOne = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .lineToYLinearHeading(redLeftStageOnePosition.position.y, redLeftStageOnePosition.heading)
                        .build()
        );
        redLeftStageTwo = new SequentialAction(
                robot.driveTrain.actionBuilder(redLeftStageOnePosition)
                        .strafeTo(redLeftStageTwoPosition.position)
                        .turnTo(redLeftStageTwoPosition.heading)
                        .build()
        );
        //

        //
        redRightStageOne = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .lineToYLinearHeading(redRightStageOnePosition.position.y, redRightStageOnePosition.heading)
                        .build()
        );
        redRightStageTwo = new SequentialAction(
                robot.driveTrain.actionBuilder(redRightStageOnePosition)
                        .strafeTo(redRightStageTwoPosition.position)
                        .turnTo(redRightStageTwoPosition.heading)
                        .build()
        );
        //

        //
        blueLeftStageOne = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .lineToYLinearHeading(blueLeftStageOnePosition.position.y, blueLeftStageOnePosition.heading)
                        .build()
        );
        blueLeftStageTwo = new SequentialAction(
                robot.driveTrain.actionBuilder(blueLeftStageOnePosition)
                        .strafeTo(blueLeftStageTwoPosition.position)
                        .turnTo(blueLeftStageTwoPosition.heading)
                        .build()
        );
        //

        //
        blueRightStageOne = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .lineToYLinearHeading(blueRightStageOnePosition.position.y, blueRightStageOnePosition.heading)
                        .build()
        );
        blueRightStageTwo = new SequentialAction(
                robot.driveTrain.actionBuilder(blueRightStageOnePosition)
                        .strafeTo(blueRightStageTwoPosition.position)
                        .turnTo(blueRightStageTwoPosition.heading)
                        .build()
        );
        //

        travelBackstageBuild();

        travelBackdrop = new SequentialAction(
                robot.driveTrain.actionBuilder(backdropApproachPosition)
                .strafeTo(backdropFinalPosition.position)
                .turnTo(standardHeading)
                .build()
        );
    }

    public void travelBackstageBuild(){
        travelBackstage = new SequentialAction(
                //robot.driveTrain.actionBuilder(purpleEndPosition)
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .turnTo(standardHeading)
                        //.splineTo(backdropApproachPosition.position, standardHeading)
                        .build()

        );
    }

    public void pickAutonToRun(Constants.Alliance alliance, int yellowOffset) {
        allianceDirection=alliance.equals(Constants.Alliance.BLUE)?-1:1;
        this.yellowOffset = yellowOffset;
        build();
            if (startingPosition == Constants.Position.START_LEFT_BLUE) {
                stageOneToRun = blueLeftStageOne;
                stageTwoToRun = blueLeftStageTwo;
            } else if (startingPosition == Constants.Position.START_RIGHT_BLUE) {
                stageOneToRun = blueRightStageOne;
                stageTwoToRun = blueRightStageTwo;
            } else if (startingPosition == Constants.Position.START_LEFT_RED) {
                stageOneToRun = redLeftStageOne;
                stageTwoToRun = redLeftStageTwo;
            } else {
                stageOneToRun = redRightStageOne;
                stageTwoToRun = redRightStageTwo;
        }
    }


    public static int autonIndex;
    public static long futureTimer;
    public static int EJECT_WAIT_TIME = 2;


    public void execute(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();

            switch (autonIndex) {
                case 0:
                    autonIndex++;
                    robot.skyhook.articulate(Skyhook.Articulation.GAME);
                    break;
                case 1:
                    autonState = AutonState.BACK_UP;
                    if (!stageOneToRun.run(packet)) {
                        robot.intake.setAngleControllerTicks(Intake.BEATER_BAR_EJECT_ANGLE);
                        autonIndex++;
                    }
                    break;
                case 2:
                    autonState = AutonState.STRAFE;
                    if (!stageTwoToRun.run(packet)) {
                        futureTimer = futureTime(EJECT_WAIT_TIME);
                        autonIndex++;
                    }
                    break;
                case 3:
                    autonState = AutonState.SCORE_GROUND;
                    robot.intake.ejectBeaterBar();
                    if (isPast(futureTimer)) {
                        robot.intake.beaterBarOff();
                        //todo - the following 2 lines should be combined into a robot.travel articulation
                        robot.intake.articulate(Intake.Articulation.TRAVEL);
                        robot.outtake.intakePosition(); //outtake should already be in this position
                        //todo IMPORTANT - lower the outtake flipper to slightly below horizontal so it goes under the door
                        travelBackstageBuild(); //gotta build again since the current position is used
                        autonIndex++;
                    }
                    break;
                case 4: //travel to interim position near backdrop and then to final position
                    autonState = AutonState.TRAVEL_BACKSTAGE;
                    if (!travelBackstage.run(packet))
                    {
                        autonIndex++;
                        autonIndex++;}

                    break;
                case 5:
                    autonState = AutonState.TRAVEL_BACKDROP;
                    if(!travelBackdrop.run(packet)) autonIndex++;
                    break;
                case 6: //todo put the outtake in a scoring config
                    autonIndex++;
                    break;
                case 7: //todo slow final approach to backdrop
                    autonIndex++;
                    break;
                case 8:
                    robot.positionCache.update(new CSPosition(robot.driveTrain.pose), true);
                    break;

            }
        dashboard.sendTelemetryPacket(packet);
    }

}
