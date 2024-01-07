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
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.CSPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.LinkedHashMap;
import java.util.Map;


@Config(value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {

    private static double APRIL_TAG_HEADING = 0;
    public VisionProvider visionProvider;
    private Robot robot;
    private HardwareMap hardwareMap;

    public enum AutonState{
        INIT,
        BACK_UP,
        STRAFE,
        SCORE_GROUND,
        FIND_STANDARD_POSITION,
        TRAVEL_BACKSTAGE,
        DONE,
        FIND_STANDARD_HEADING,
        TRAVEL_BACKDROP,
        ALIGN_WITH_APRILTAG,
        FIND_APRIL_TAG_HEADING, PREP_FOR_PARK, PARK, SCORE_DRIVE, SCORE_BACKDROP
    }

    public AutonState autonState = AutonState.INIT;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("autonState", autonState);
        telemetryMap.put("targetIndex", targetIndex);
        telemetryMap.put("targetAprilTag", targetAprilTagIndex);
//        telemetryMap.put("indexStrafeOffset", indexStrafeOffset);
//        telemetryMap.put("indexHeadingOffset", indexHeadingOffset);
        telemetryMap.put("visionProvider name", visionProvider.getTelemetryName());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTONOMOUS";
    }

    // autonomous routines

    public static int targetIndex = 1;
    public static int targetAprilTagIndex = 1;
    public static int visionProviderIndex;
    public static double aprilTagOffset = .2;
    int allianceDirection = -1;

    double STANDARD_HEADING = 180;
    Pose2d actionEndPose = new Pose2d(0, 0, 0);
    private Action
            redLeftStageOne, redLeftStageTwo,
            blueLeftStageOne, blueLeftStageTwo,
            blueRightStageOne, blueRightStageTwo,
            redRightStageOne, redRightStageTwo,
            findStandardPosition, approachBackdrop,
            approachAprilTag, findStandardHeading,
            findAprilTagHeading, park;
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

    Pose2d aprilTagApproachPosition;
    Pose2d parkPosition;


    public static int INWARD_SCORING_ANGLE = -45;
    public static int MIDDLE_SCORING_ANGLE = -50;
    public static int OUTWARD_SCORING_ANGLE = 30;
    public static double indexStrafeOffset = 0;

    //BASED ON BLUE_RIGHT_START
    public static double STAGE_ONE_Y_COORDINATE = .4;

    public static double STAGE_TWO_X_COORDINATE = -2.1;
    public static double STAGE_TWO_HEADING = 40;
    public static double STAGE_ONE_HEADING = 90;
    public static double BACKSTAGE_X_POSITION_OFFSET = 2.5;

    public void updateIndexOffsets() {
        visionProviderIndex = robot.visionProviderBack.getMostFrequentPosition().getIndex();
        targetIndex = visionProviderIndex + 1;

        if(startingPosition.equals(Constants.Position.START_RIGHT_BLUE)) {
            targetAprilTagIndex = targetIndex;
            if (targetIndex == 3) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = 0.65;
                STAGE_TWO_X_COORDINATE = -1.65;
                STAGE_TWO_HEADING = 90 + INWARD_SCORING_ANGLE;
            }

            if (targetIndex == 2) {
                STAGE_ONE_HEADING = 90;

                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_HEADING = 90 + MIDDLE_SCORING_ANGLE;
                STAGE_TWO_X_COORDINATE = -2.1;
            }

            if (targetIndex == 1) {


                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_X_COORDINATE = -1.55;
                STAGE_TWO_HEADING = 90 + OUTWARD_SCORING_ANGLE;
            }
        }

        if(startingPosition.equals(Constants.Position.START_LEFT_BLUE)) {
            targetAprilTagIndex = targetIndex;
            if (targetIndex == 1) {
                STAGE_ONE_HEADING = 90;

                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_X_COORDINATE = 0.55;
                STAGE_TWO_HEADING = 90 - OUTWARD_SCORING_ANGLE;
            }

            if (targetIndex == 2) {
                STAGE_ONE_HEADING = 90;

                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_HEADING = 90 - MIDDLE_SCORING_ANGLE;
                STAGE_TWO_X_COORDINATE = 1.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_HEADING = 90;

                STAGE_ONE_Y_COORDINATE = 0.65;
                STAGE_TWO_X_COORDINATE = 0.65;
                STAGE_TWO_HEADING = 90 - INWARD_SCORING_ANGLE;
            }
        }

        if(startingPosition.equals(Constants.Position.START_LEFT_RED)) {
            targetAprilTagIndex = targetIndex + 3;
            if (targetIndex == 1) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -0.65;
                STAGE_TWO_X_COORDINATE = -1.65;
                STAGE_TWO_HEADING = (90 + OUTWARD_SCORING_ANGLE);
            }

            if (targetIndex == 2) {
                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_HEADING = (90 + MIDDLE_SCORING_ANGLE);
                STAGE_TWO_X_COORDINATE = -2.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_X_COORDINATE = -1.55;
                STAGE_TWO_HEADING = (90 + INWARD_SCORING_ANGLE);
            }
        }
        if(startingPosition.equals(Constants.Position.START_RIGHT_RED)) {
            targetAprilTagIndex = targetIndex + 3;
            if (targetIndex == 1) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -0.65;
                STAGE_TWO_X_COORDINATE = 0.65;
                STAGE_TWO_HEADING = (90 - INWARD_SCORING_ANGLE);
            }

            if (targetIndex == 2) {
                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_HEADING = (90 - MIDDLE_SCORING_ANGLE);
                STAGE_TWO_X_COORDINATE = 1.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_X_COORDINATE = 0.55;
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

        purpleEndPosition = P2D(startingPosition.getPose().position.x / FIELD_INCHES_PER_GRID, allianceDirection * .35 * FIELD_INCHES_PER_GRID, STANDARD_HEADING);
        backdropApproachPosition = P2D(1.65, allianceDirection * .35, STANDARD_HEADING);
        aprilTagApproachPosition = P2D(1.8, allianceDirection * 1.5 + (targetAprilTagIndex - 3) * aprilTagOffset, STANDARD_HEADING);

        parkPosition = P2D(1.6, allianceDirection * 2.1, STANDARD_HEADING);

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
        approachAprilTagBuild();
        findStandardPositionBuild();
        approachBackdropBuild();


    }

    public void approachBackdropBuild() {
        approachBackdrop = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .turnTo(purpleEndPosition.heading)
                        .strafeTo(backdropApproachPosition.position)
                        .build()
        );
    }

    public void findStandardPositionBuild(){
        findStandardPosition = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .turnTo(purpleEndPosition.heading)
                        .strafeTo(purpleEndPosition.position)
                        .build()

        );
    }

    public void parkBuild() {
        park = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .lineToYConstantHeading(parkPosition.position.y)
                        .build()
        );
    }

    public void findStandardHeadingBuild() {
        findStandardHeading = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .turnTo(Math.toRadians(STANDARD_HEADING))
                        .build()
        );
    }

    public void findAprilTagHeadingBuild() {
        APRIL_TAG_HEADING = Math.atan2(aprilTagApproachPosition.position.y-robot.driveTrain.pose.position.y,aprilTagApproachPosition.position.x-robot.driveTrain.pose.position.x);
        findAprilTagHeading = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .turnTo(Math.toRadians(APRIL_TAG_HEADING))
                        .build()
        );
    }

    public void approachAprilTagBuild() {
        approachAprilTag = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .splineToLinearHeading(aprilTagApproachPosition,Math.PI)
                        .build()
        );
    }

    public void pickAutonToRun(Constants.Alliance alliance) {
        allianceDirection=alliance.equals(Constants.Alliance.BLUE)?1:-1;
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



    public boolean execute(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();

            switch (autonIndex) {
                case 0:
                    futureTimer = futureTime(.4);
                    autonIndex++;
                    break;
                case 1:
                    autonState = AutonState.BACK_UP;
                    if (!stageOneToRun.run(packet)) {
                        autonIndex++;
                    } else if (isPast(futureTimer)) {
                        //putting the intake down to eject position so it can nudge the team prop out of the way for the 1 and 3 positions
                        //doesn't matter if it gets set repeatedly
                        robot.intake.setAngle(Intake.ANGLE_EJECT);
                    }
                    break;
                case 2:
                    autonState = AutonState.STRAFE;
                    if (!stageTwoToRun.run(packet)) {
                        robot.intake.articulate(Intake.Articulation.EJECT);
                        //reset roadrunner based on imuAngle
                        robot.driveTrain.pose = new Pose2d(robot.driveTrain.pose.position, Math.toRadians(robot.driveTrain.imuAngle));
                        autonIndex++;
                    }
                    break;
                case 3:
                    autonState = AutonState.SCORE_GROUND;
                    if(robot.intake.readyForTravel()) {
                        autonIndex++;
                    }
                    break;
                case 4:
                    if(robot.outtake.ingestFromTravel()) //todo should call an articulation instead
                        {
                            findStandardPositionBuild(); //gotta build again since the current position is used
                            autonIndex++;
                        }
                    break;
                case 5: //travel to interim position near backdrop and then to final position
                    if(startingPosition != Constants.Position.START_RIGHT_BLUE && startingPosition != Constants.Position.START_LEFT_RED) {
                        autonIndex++;
                        break;
                    }
                    autonState = AutonState.FIND_STANDARD_POSITION;
                    if (!findStandardPosition.run(packet)) {
                        approachBackdropBuild();
                        autonIndex++;
                    }
                    break;
                case 6:
                    if(startingPosition != Constants.Position.START_RIGHT_BLUE && startingPosition != Constants.Position.START_LEFT_RED) {
                        findAprilTagHeadingBuild();
                        autonIndex++;
                        break;
                    }
                    autonState = AutonState.TRAVEL_BACKDROP;
                    if(!approachBackdrop.run(packet)) {
                        findAprilTagHeadingBuild();
                        autonIndex++;
                    }
                    break;
                case 7:
                    autonState = AutonState.FIND_APRIL_TAG_HEADING;
                    if(!findAprilTagHeading.run(packet)) {
                        approachAprilTagBuild();
                        autonIndex++;
                    }
                    break;
                case 8:
                    autonState = AutonState.ALIGN_WITH_APRILTAG;
                    if(!approachAprilTag.run(packet)) {
                        actionEndPose = robot.driveTrain.pose;
                        //reset roadrunner based on imuAngle
                        robot.driveTrain.pose = new Pose2d(robot.driveTrain.pose.position, Math.toRadians(robot.driveTrain.imuAngle));
                        findStandardHeadingBuild();

                        autonIndex++;
                    }
                    break;
                case 9:
                    autonState = AutonState.FIND_STANDARD_HEADING;
                    if(!findStandardHeading.run(packet)) {
                        robot.articulate(Robot.Articulation.BACKDROP_PREP);
                        autonIndex++;
                    }
                    break;
                case 10:
                    autonState = AutonState.SCORE_BACKDROP;
                    if(robot.outtake.articulation.equals(Outtake.Articulation.BACKDROP))
                        autonIndex++;
                    break;
                case 11:
                    autonState = AutonState.SCORE_DRIVE;
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.4, 0), 0));
                    if(robot.driveTrain.backDistanceSensor.getDistance(DistanceUnit.INCH) < 8.5) {
                        robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        autonIndex++;
                    }
                    break;
                case 12:
//                    autonState = AutonState.PREP_FOR_PARK;
//                    robot.outtake.setTargetAngle(Outtake.FLIPPER_PRE_SCORE_ANGLE);
//                    parkBuild();
                    autonIndex ++;
                    break;
                case 13:
//                    autonState = AutonState.PARK;
//                    if(!park.run(packet)) {
                        autonIndex++;
//                    }
                    break;
                case 14:
                    autonState = AutonState.DONE;
                    robot.positionCache.update(new CSPosition(robot.driveTrain.pose, robot.skyhook.getSkyhookLeftTicksCurrent(), robot.skyhook.getSkyhookRightTicksCurrent()), true);
                    return true;

            }
            dashboard.sendTelemetryPacket(packet);
            return false;
    }

}
