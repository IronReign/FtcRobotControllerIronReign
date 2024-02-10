package org.firstinspires.ftc.teamcode.robots.goldenduck;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.FIELD_INCHES_PER_GRID;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.P2D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;
import org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.CSPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.LinkedHashMap;
import java.util.Map;

//todo this entire set of sequences is ripped from Iron Reign
//todo will need to rebuild
@Config(value = "GD_Auton")
public class Autonomous implements TelemetryProvider {

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
        TRAVEL_BACKDROP,
        ALIGN_WITH_APRILTAG,
        SCORE_BACKDROP
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
    private Action
            redLeftStageOne, redLeftStageTwo,
            blueLeftStageOne, blueLeftStageTwo,
            blueRightStageOne, blueRightStageTwo,
            redRightStageOne, redRightStageTwo,
            findStandardPosition, approachBackdrop,
            approachAprilTag
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

        purpleEndPosition = P2D(startingPosition.getPose().position.x / FIELD_INCHES_PER_GRID, allianceDirection * .35, STANDARD_HEADING);
        backdropApproachPosition = P2D(1.65, allianceDirection * .35, STANDARD_HEADING);
        aprilTagApproachPosition = P2D(1.65, allianceDirection * 2 + (targetAprilTagIndex - 3) * aprilTagOffset, STANDARD_HEADING);

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

    public void approachAprilTagBuild() {
        approachAprilTag = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .strafeTo(aprilTagApproachPosition.position)
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
    public static long targetTicsLeftOdo; //for simple distance calcs between moves
    public double targetAngle;


    public boolean execute(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();

            switch (autonIndex) {
                case 0:
                    targetTicsLeftOdo = robot.driveTrain.getLeftOdo() + robot.driveTrain.distInTics(26);
                    robot.driveTrain.imu.resetYaw();
                    //drive forward really slowly
                    robot.driveTrain.drive(0,-.3,0);
                    //set arm down
                    robot.arm.setShoulderTargetPosition(500); //between intake and travel
                    autonIndex++;
                    break;
                case 1: //drive to purple pixel drop off location
                    autonState = AutonState.BACK_UP;
                    if (robot.driveTrain.getLeftOdo()>targetTicsLeftOdo) {
                        robot.driveTrain.drive(0, 0, 0);
                        targetAngle = Math.toDegrees(startingPosition.getPose().heading.log());
                        if (targetIndex == 3) { //turn left
                            robot.driveTrain.drive(0, 0, .2);
                            targetAngle = 0;
                        }
                        if (targetIndex == 1) { //turn right
                            robot.driveTrain.drive(0, 0, -.2);
                            targetAngle =  180;
                        }
                        autonIndex ++;
                    }
                    break;
                case 2: //if target angle is good enough drop the pixel
                    if (withinError(Utils.wrapAngle(robot.driveTrain.imuAngle), targetAngle, 2)) //within 2 degrees of target
                    {
                        robot.driveTrain.drive(0, 0, 0);
                        //todo something here to drop the purple pixel
                        robot.arm.GripInnerOnly(); //purple needs to be on the outer/lower position
                        autonIndex++;
                        return true;
                    }
                    //autonState = AutonState.STRAFE;
//                    if (!stageTwoToRun.run(packet)) {
//                        //todo something here to drop the purple pixel
//                        robot.arm.GripInnerOnly(); //purple needs to be on the outer/lower position
//                        autonIndex++;
//                    }
                    break;
                case 3:
                    autonState = AutonState.SCORE_GROUND;
                    //back up out of the way
//                    if(robot.intake.readyForTravel()) {
//                        autonIndex++;
//                    }
                    break;
                case 4:
                    autonIndex++;
                    break;
                case 5: //travel to interim position near backdrop and then to final position?
                    autonIndex++;
                    break;
                case 6:
                    autonState = AutonState.TRAVEL_BACKDROP;
                    if(!approachBackdrop.run(packet)) {
                        approachAprilTagBuild();
                        autonIndex++;
                    }
                    break;
                case 7:
                    autonState = AutonState.ALIGN_WITH_APRILTAG;
                    if(!approachAprilTag.run(packet)) {
                        autonIndex++;
                    }
                    break;
                case 8:
                    autonState = AutonState.DONE;
                    autonIndex++;
                    break;
                case 9:
                    robot.positionCache.update(new CSPosition(robot.driveTrain.pose, 0, 0), true);
                    return true;

            }
            dashboard.sendTelemetryPacket(packet);
            return false;
    }

    public static boolean withinError(double value, double target, double error){
        return (Math.abs(target-value) <= error);
    }

}
