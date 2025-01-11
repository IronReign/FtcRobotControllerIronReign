package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.frontAuton;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.DriveTrain.runTestPath;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.DriveTrain.testPathToScore;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.DriveTrain.testPathToWing;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.DriveTrain.turnToSpeed;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake.ELBOW_TRAVEL_ANGLE;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake.ELEVATOR_START_ANGLE;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake.WRIST_TRAVEL_ANGLE;
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

import org.firstinspires.ftc.teamcode.rrQuickStart.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Sensors;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Skyhook;
import org.firstinspires.ftc.teamcode.robots.csbot.util.CSPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.AprilTagProvider;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.LinkedHashMap;
import java.util.Map;


@Config(value = "CS_Auton")
public class Autonomous implements TelemetryProvider {

    public static double SCORE_DRIVE_DISTANCE = 3;
    private Robot robot;
    private HardwareMap hardwareMap;

    public enum AutonState {
        INIT,
        TRAVEL_TO_PURPLE,
        STRAFE,
        SCORE_GROUND,
        FIND_STANDARD_POSITION,
        TRAVEL_BACKSTAGE,
        DONE,
        FIND_STANDARD_HEADING,
        TRAVEL_BACKDROP,
        ALIGN_WITH_APRILTAG,
        FIND_APRIL_TAG_HEADING, PREP_FOR_PARK, PARK, SCORE_DRIVE, DRIVE_TO_PIXEL_STACK, GET_FROM_PIXEL_STACK, APRILTAG_STRAFE, IMU_TURN, APRILTAG_RELOCALIZE, SCORE_BACKDROP
    }

    public AutonState autonState = AutonState.INIT;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("autonState", autonState);
        telemetryMap.put("auton index", autonIndex);
        telemetryMap.put("targetIndex", targetIndex);
        telemetryMap.put("targetAprilTag", targetAprilTagIndex);
        telemetryMap.put("selectedPath", selectedPath);
        telemetryMap.put("front auton", frontAuton);
        telemetryMap.put("driverside?", Constants.driverSide);
        telemetryMap.put("to pixel stack?", Constants.runPixelStack);
        if (autonState == AutonState.ALIGN_WITH_APRILTAG) {
            telemetryMap.put("apriltag index", ((AprilTagProvider) robot.visionProviderBack).getIndex());
        }
//        telemetryMap.put("indexStrafeOffset", indexStrafeOffset);
//        telemetryMap.put("indexHeadingOffset", indexHeadingOffset);
        telemetryMap.put("visionProvider name", robot.visionProviderBack.getTelemetryName());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTONOMOUS";
    }

    // autonomous routines

    public int targetIndex = 1;
    public static int targetAprilTagIndex = 1;
    public double aprilTagOffset = .2;
    int allianceDirection = -1;
    public static int selectedPath;
    double STANDARD_HEADING_RAD = Math.PI;
    boolean testRunToWing = true;

    public static double FIELD_INCHES_PER_GRID = 23.5;
    public static double AUTON_START_DELAY = 0;

    double STANDARD_HEADING = 180;
    Pose2d aprilTagApproachPosition;
    Pose2d audienceIntermediate;
    Pose2d driverSidePrep, driverSidePrepForward;
    Pose2d audienceIntermediateForward, audienceIntermediateDeep, pixelStackAudienceIntermediate, pixelStackAudienceIntermediateForward;
    Pose2d aprilTagAlign, aprilTagAlignClose, aprilTagAlignCrossed;
    Pose2d parkAudience, parkBackDrop;
    Pose2d pixelStack;


    //values to actually use
    Pose2d[][] autonPaths;

    private Action
            driveToPurplePixel,
            driveToYellowPixel,
            aprilTagStrafe,
            approachBackdrop,
            driveToPixelStack1,
            driveToPixelStack2,
            adjustForStrafe,
            strafeToPark;

    // misc. routines
    public StateMachine backAndForth, square, turn;

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        autonPaths = new Pose2d[7][13];
        autonIndex = 0;
    }

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

    public void driveToYellowPixelBuild() {
        if (!Constants.driverSide) {
            driveToYellowPixel = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .setReversed(true)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .build()
            );
        } else {
            driveToYellowPixel = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                            .setReversed(true)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][5].position), STANDARD_HEADING_RAD)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][6].position), STANDARD_HEADING_RAD)
                            .build()
            );
        }
    }

    public void approachBackdropBuild() {
        if (!Constants.driverSide) {
            approachBackdrop = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .setReversed(true)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .build()
            );
        } else {
            approachBackdrop = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .setReversed(true)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][10].position), STANDARD_HEADING_RAD)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][11].position), STANDARD_HEADING_RAD)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][12].position), STANDARD_HEADING_RAD)
                            .build()
            );
        }
    }

    public void adjustForStrafeBuild() {
        //we need to force the turn in one direction for these paths
        if (!Constants.driverSide) {
                adjustForStrafe = new SequentialAction(
                        robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                                .strafeToLinearHeading(switchSides(autonPaths[selectedPath][3].position), switchSides(autonPaths[selectedPath][3].heading.log()))
                                .build());
        } else {
            adjustForStrafe = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .build()
            );
        }

    }

    public void driveToPurplePixelBuild() {
        if (!Constants.driverSide) {
            driveToPurplePixel = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                            .build());
        } else {
            driveToPurplePixel = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                            .build());

        }
    }
    public void driveToPixelStack1Build() {
        if (!Constants.driverSide) {
            driveToPixelStack1 = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .setReversed(false)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .build()
            );
        } else {
            driveToPixelStack1 = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .setReversed(false)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][7].position), STANDARD_HEADING_RAD)
                            .build()
            );
        }
    }

    public void driveToPixelStack2Build() {
        if (!Constants.driverSide) {
            driveToPixelStack2 = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .setReversed(false)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][7].position), switchSides(autonPaths[selectedPath][7].heading.log()))
                            .build()
            );
        } else {
            driveToPixelStack2 = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .setReversed(false)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][8].position), STANDARD_HEADING_RAD)
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][9].position), STANDARD_HEADING_RAD)
                            .build()
            );
        }
    }

    public void strafeToParkBuild() {
        if (!Constants.driverSide) {
            strafeToPark = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .strafeTo(new Vector2d(robot.driveTrain.pose.position.x, switchSides(autonPaths[selectedPath][5].position).y+20*allianceDirection))
                            .build()
            );
        } else {
            strafeToPark = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .strafeTo(new Vector2d(robot.driveTrain.pose.position.x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection))
                            .build()
            );
        }
    }

    public void aprilTagStrafeBuild() {
        aprilTagStrafe = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .strafeTo(new Vector2d(robot.driveTrain.pose.position.x, POI.getAprilTag(targetAprilTagIndex).getPose().position.y))
                        .build()
        );
    }

    public void saveRandomizer(int visionIndex) {
        targetIndex = visionIndex + 1;

    }

    public void pickAutonToRun(Constants.Position startingPosition) {
        selectedPath = setPath(startingPosition, targetIndex, Constants.driverSide);
    }

    public static Pose2d P2D(double x, double y, double deg) {
        return new Pose2d(x * FIELD_INCHES_PER_GRID, y * FIELD_INCHES_PER_GRID, Math.toRadians(deg));
    }

    public Pose2d switchSides(Pose2d p) {
        return new Pose2d(p.position.x, allianceDirection * p.position.y, -allianceDirection * p.heading.log());
    }

    public Vector2d switchSides(Vector2d v) {
        return new Vector2d(v.x, allianceDirection * v.y);
    }

    public double switchSides(double r) {
        return -allianceDirection * r;
    }

    public int setPath(Constants.Position startingPosition, int randomizer, boolean driverSide) { // 1, 2 or 3 for randomized prop
        if (randomizer == 0)
            randomizer = 2;
//        if(randomizer == 3){
//            randomizer = 1;
//        }
//        if(randomizer == 1){
//            randomizer = 3;
//        }
        targetAprilTagIndex = alliance.isRed() ? 3 + randomizer : randomizer;
        if (!driverSide) {
            aprilTagApproachPosition = P2D(1.5, 1.5, STANDARD_HEADING);
            audienceIntermediate = P2D(1.5, .5, -10);
            audienceIntermediateForward = P2D(1.5, .5, STANDARD_HEADING);
            audienceIntermediateDeep = P2D(1.5, .5, -10);
            allianceDirection = startingPosition.isRed() ? -1 : 1;
            parkAudience = P2D(2.5, -0.5, STANDARD_HEADING);
            parkBackDrop = P2D(2, -2.45, STANDARD_HEADING);

            //aprilTagAlign = new Pose2d (new Vector2d(switchSides(aprilTagApproachPosition.position).x,switchSides(aprilTagApproachPosition.position).y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x-1,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
            aprilTagAlign = new Pose2d(new Vector2d(aprilTagApproachPosition.position.x, aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)), STANDARD_HEADING_RAD);
            aprilTagAlignClose = new Pose2d(new Vector2d(aprilTagApproachPosition.position.x - 1, aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)),
                    0);
            aprilTagAlignCrossed = new Pose2d(new Vector2d(aprilTagApproachPosition.position.x, aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)), STANDARD_HEADING_RAD);
            pixelStack = P2D(-2.25, .5, STANDARD_HEADING);

            //assemble the paths
            autonPaths[1][1] = P2D(-2, .5, -90);
            autonPaths[1][2] = P2D(0, 0, -90);
            autonPaths[1][3] = P2D(-1.7, .5, STANDARD_HEADING);
            autonPaths[1][4] = audienceIntermediate;
            autonPaths[1][5] = aprilTagAlign;
            autonPaths[1][6] = audienceIntermediateForward;
            autonPaths[1][7] = pixelStack;
            autonPaths[1][8] = audienceIntermediate;
            autonPaths[1][9] = aprilTagAlign;

            autonPaths[2][1] = P2D(-1.5, .3, -90);
            autonPaths[2][2] = P2D(0, 0, -90);
            autonPaths[2][3] = P2D(-1.7, .5, STANDARD_HEADING);
            autonPaths[2][4] = audienceIntermediate;
            autonPaths[2][5] = aprilTagAlign;
            autonPaths[2][6] = audienceIntermediateForward;
            autonPaths[2][7] = pixelStack;
            autonPaths[2][8] = audienceIntermediate;
            autonPaths[2][9] = aprilTagAlign;

            autonPaths[3][1] = P2D(-1.5, 1, -30);
            autonPaths[3][2] = P2D(0, 0, -30);
            autonPaths[3][3] = P2D(-1.7, .5, STANDARD_HEADING);
            autonPaths[3][4] = audienceIntermediateDeep;
            autonPaths[3][5] = aprilTagAlignCrossed;
            autonPaths[3][6] = audienceIntermediateForward;
            autonPaths[3][7] = pixelStack;
            autonPaths[3][8] = audienceIntermediate;
            autonPaths[3][9] = aprilTagAlign;

            autonPaths[4][1] = P2D(startingPosition.getPose().position.x/FIELD_INCHES_PER_GRID+.2, 1.25, STANDARD_HEADING);
            autonPaths[4][2] = P2D(0, 0, STANDARD_HEADING);
            autonPaths[4][3] = P2D(-1.7, .5, STANDARD_HEADING);
            autonPaths[4][4] = aprilTagAlignClose;
            autonPaths[4][5] = aprilTagAlign;
            autonPaths[4][6] = audienceIntermediateForward;
            autonPaths[4][7] = pixelStack;
            autonPaths[4][8] = audienceIntermediate;
            autonPaths[4][9] = aprilTagAlign;

            autonPaths[5][1] = P2D(startingPosition.getPose().position.x/FIELD_INCHES_PER_GRID, 1.7, 90);
            autonPaths[5][2] = P2D(0, 0, 170);
            autonPaths[5][3] = P2D(-1.7, .5, STANDARD_HEADING);
            autonPaths[5][4] = aprilTagAlignClose;
            autonPaths[5][5] = aprilTagAlign;
            autonPaths[5][6] = audienceIntermediateForward;
            autonPaths[5][7] = pixelStack;
            autonPaths[5][8] = audienceIntermediate;
            autonPaths[5][9] = aprilTagAlign;

            autonPaths[6][1] = P2D(.4, 1.75 , 35);
            autonPaths[6][2] = P2D(0, 0, STANDARD_HEADING);
            autonPaths[6][3] = P2D(-1.7, .5, STANDARD_HEADING);
            autonPaths[6][4] = aprilTagAlignClose;
            autonPaths[6][5] = aprilTagAlign;
            autonPaths[6][6] = audienceIntermediateForward;
            autonPaths[6][7] = pixelStack;
            autonPaths[6][8] = audienceIntermediate;
            autonPaths[6][9] = aprilTagAlign;
        } else {
            aprilTagApproachPosition = P2D(1.5,1.5, STANDARD_HEADING);
            audienceIntermediate = P2D(1,.5,-10);
            pixelStackAudienceIntermediate = P2D(driverSide?-1.5:1,driverSide?2.5:.5,-10);
            pixelStackAudienceIntermediateForward = P2D(driverSide?1:1,driverSide?2.5:.3,-10);
            audienceIntermediateForward = P2D(driverSide?-1.5:1.4, driverSide?2.5:.5, STANDARD_HEADING);
            driverSidePrep = P2D(1, 2.5, -10);
            driverSidePrepForward = P2D(1, 2.5, STANDARD_HEADING);
            audienceIntermediateDeep = P2D(driverSide?1:1.5,driverSide?2.5:.5,-10);
            allianceDirection = startingPosition.isRed()? -1 : 1;

            //aprilTagAlign = new Pose2d (new Vector2d(switchSides(aprilTagApproachPosition.position).x,switchSides(aprilTagApproachPosition.position).y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x-1,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
            aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)), 0);
            aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x - 1,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)),
                    0);
            aprilTagAlignCrossed = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)), Math.toRadians(-90));
            pixelStack = P2D(-2.25, driverSide?1.5:.5, STANDARD_HEADING);

            //assemble the paths
            autonPaths[1][1] = P2D(-1.9, driverSide?1.9:.5, 90);
            autonPaths[1][2] = P2D(0, 0, driverSide?90:-90);
            autonPaths[1][3] = P2D(0, 0, driverSide?-90:STANDARD_HEADING);
            autonPaths[1][4] = P2D(-2, driverSide?2.5:.5, STANDARD_HEADING);
            autonPaths[1][5] = pixelStackAudienceIntermediateForward;
            autonPaths[1][6] = aprilTagAlign;
            autonPaths[1][7] = driverSidePrepForward;
            autonPaths[1][8] = audienceIntermediateForward;
            autonPaths[1][9] = pixelStack;
            autonPaths[1][10] = pixelStackAudienceIntermediate;
            autonPaths[1][11] = driverSidePrep;
            autonPaths[1][12] = aprilTagAlign;

            autonPaths[2][1] = P2D(driverSide?-1.5:-1.2, driverSide?1.75:.43, 90);
            autonPaths[2][2] = P2D(0, 0, driverSide?90:-90-35);
            autonPaths[2][3] = P2D(0, 0, driverSide?-90:STANDARD_HEADING);
            autonPaths[2][4] = P2D(-2, 2.5, STANDARD_HEADING);
            autonPaths[2][5] = pixelStackAudienceIntermediateForward;
            autonPaths[2][6] = aprilTagAlign;
            autonPaths[2][7] = driverSidePrepForward;
            autonPaths[2][8] = audienceIntermediateForward;
            autonPaths[2][9] = pixelStack;
            autonPaths[2][10] = pixelStackAudienceIntermediate;
            autonPaths[2][11] = driverSidePrep;
            autonPaths[2][12] = aprilTagAlign;
            System.out.println(switchSides(aprilTagAlign.position).y);

            autonPaths[3][1] = P2D(-1.6, driverSide?1.75:1, driverSide?25:90);
            autonPaths[3][2] = P2D(0, 0, driverSide?45:-30);
            autonPaths[3][3] = P2D(0, 0, -130);
            autonPaths[3][4] = P2D(-2, 2.5, STANDARD_HEADING);
            autonPaths[3][5] = audienceIntermediateDeep;
            autonPaths[3][6] = driverSide?aprilTagAlign:aprilTagAlignCrossed;
            autonPaths[3][7] = driverSidePrepForward;
            autonPaths[3][8] = audienceIntermediateForward;
            autonPaths[3][9] = pixelStack;
            autonPaths[3][10] = pixelStackAudienceIntermediate;
            autonPaths[3][11] = driverSidePrep;
            autonPaths[3][12] = aprilTagAlign;

            autonPaths[4][1] = P2D((startingPosition.getPose().position.x/FIELD_INCHES_PER_GRID+.2), 1.25, STANDARD_HEADING);
            autonPaths[4][2] = P2D(0, 0, STANDARD_HEADING);
            autonPaths[4][3] = P2D(0, 0, STANDARD_HEADING);
            autonPaths[4][4] = P2D(.81, 1.31, 180);
            autonPaths[4][5] = aprilTagAlignClose;
            autonPaths[4][6] = aprilTagAlign;
            autonPaths[4][7] = driverSidePrepForward;
            autonPaths[4][8] = audienceIntermediateForward;
            autonPaths[4][9] = pixelStack;
            autonPaths[4][10] = pixelStackAudienceIntermediate;
            autonPaths[4][11] = driverSidePrep;
            autonPaths[4][12] = aprilTagAlign;

            autonPaths[5][1] = P2D(driverSide?.5:startingPosition.getPose().position.x/FIELD_INCHES_PER_GRID, 1.7, 90);
            autonPaths[5][2] = P2D(0, 0, 90);
            autonPaths[5][3] = P2D(0, 0, 170);
            autonPaths[5][4] = P2D(.51, 1.81, 170);
            autonPaths[5][5] = aprilTagAlignClose;
            autonPaths[5][6] = aprilTagAlign;
            autonPaths[5][7] = driverSidePrepForward;
            autonPaths[5][8] = audienceIntermediateForward;
            autonPaths[5][9] = pixelStack;
            autonPaths[5][10] = pixelStackAudienceIntermediate;
            autonPaths[5][11] = driverSidePrep;
            autonPaths[5][12] = aprilTagAlign;

            autonPaths[6][1] = P2D(.4, 1.75 , 35);
            autonPaths[6][2] = P2D(0, 0, driverSide?45:125);
            autonPaths[6][3] = P2D(0, 0, STANDARD_HEADING);
            autonPaths[6][4] = P2D(.51, 1.76, STANDARD_HEADING);
            autonPaths[6][5] = aprilTagAlignClose;
            autonPaths[6][6] = aprilTagAlign;
            autonPaths[6][7] = driverSidePrepForward;
            autonPaths[6][8] = audienceIntermediateForward;
            autonPaths[6][9] = pixelStack;
            autonPaths[6][10] = pixelStackAudienceIntermediate;
            autonPaths[6][11] = driverSidePrep;
            autonPaths[6][12] = aprilTagAlign;
        }

        int rando = randomizer;
        if (allianceDirection == 1 && randomizer == 1) rando = 3;
        if (allianceDirection == 1 && randomizer == 3) rando = 1;
        return (startingPosition.equals(Constants.Position.START_RIGHT_RED) || startingPosition.equals(Constants.Position.START_LEFT_BLUE)) ?
                3 + rando : rando;
    }

    public static int autonIndex;
    public long futureTimer = futureTime(10);

    int firstIndex = 0;


    public boolean execute(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();
        if (runTestPath) {
            if (testRunToWing) {
                if (!testPathToWing.run(packet)) {
                    robot.driveTrain.buildTestPathToScore();
                    testRunToWing = false;
                }
            } else {
                if (!testPathToScore.run(packet)) {
                    robot.driveTrain.buildTestPathToWing();
                    testRunToWing = true;
                }
            }
        } else
            switch (autonIndex) {
                case 0:
                    Robot.frontVision = false; //could be front or back during init, but this is what'll be used for apriltag
                    Robot.backVisionProviderIndex = 0; // forcing apriltag & refinalization
                    robot.visionProviderFinalized = false;
                    Sensors.distanceSensorsEnabled = false;
                    driveToPurplePixelBuild();
                    robot.outtake.articulate(Outtake.Articulation.START);
                    robot.intake.articulate(Intake.Articulation.INIT);
                    futureTimer = futureTime(AUTON_START_DELAY);//delay for auton start
                    autonIndex++;
                    break;
                case 1:
                    autonState = AutonState.TRAVEL_TO_PURPLE;
                    if(!frontAuton) {
                        robot.intake.articulate(Intake.Articulation.INIT);
                    }
                    else robot.intake.articulate(Intake.Articulation.DOWN);
                    if (isPast(futureTimer)) {
                        if (!driveToPurplePixel.run(packet)) {
                            robot.skyhook.articulate(Skyhook.Articulation.TRAVEL);
                            autonIndex++;
                            break;
                        }
                    }
                    break;
                case 2:
                    autonState = AutonState.SCORE_GROUND;
                    robot.outtake.elbow.setSpeed(Outtake.ELBOW_JOINT_SPEED * .5);
                    robot.intake.articulate(Intake.Articulation.EJECT);
                    adjustForStrafeBuild();
                    autonIndex++;
                    break;
                case 3:
                    if(robot.intake.readyForTravel() && frontAuton) {
                        robot.outtake.setTargetAngle(ELBOW_TRAVEL_ANGLE, WRIST_TRAVEL_ANGLE, ELEVATOR_START_ANGLE);
                        driveToYellowPixelBuild();
                        autonIndex++;
                    }
                    else if (robot.intake.readyForTravel() /*&& robot.driveTrain.turnUntilDegreesIMU(STANDARD_HEADING, turnToSpeed)!adjustForStrafe.run(packet)*/) {
                        robot.outtake.setTargetAngle(ELBOW_TRAVEL_ANGLE, WRIST_TRAVEL_ANGLE, ELEVATOR_START_ANGLE);
                        driveToYellowPixelBuild();
                        autonIndex++;
                    }
                    break;
                case 4:
                    robot.enableVision();
                    autonIndex++;
                    break;
                case 5:
                    robot.enableVision();
                    autonState = AutonState.TRAVEL_BACKDROP;
                    if (!driveToYellowPixel.run(packet)) {
                        futureTimer = futureTime(0);
                        autonIndex++;
                    }
                    break;

                case 6:
                    autonState = AutonState.IMU_TURN;
                    if (robot.driveTrain.turnUntilDegreesIMU(STANDARD_HEADING, turnToSpeed) || isPast(futureTimer)) {
                        robot.enableVision();
                        robot.driveTrain.setPose(new Pose2d(new Vector2d(robot.driveTrain.pose.position.x, robot.driveTrain.pose.position.y), Math.toRadians(robot.sensors.driveIMUYaw)));
                        autonIndex++;
                    }
                    break;
                case 7:
                    autonState = AutonState.APRILTAG_RELOCALIZE;
                    robot.enableVision();
                    futureTimer = futureTime(20000);
                    autonIndex++;
                    break;
                case 8:
                    robot.enableVision();
                    if (isPast(futureTimer) || robot.getAprilTagDetections() != null) {
                        robot.aprilTagRelocalization(targetAprilTagIndex);
                        robot.switchVisionProviders();
                        aprilTagStrafeBuild();
                        autonIndex++;
                    }
                    break;
                case 9:
                    autonState = AutonState.APRILTAG_STRAFE;
                    if (!aprilTagStrafe.run(packet)) {
                        robot.articulate(Robot.Articulation.BACKDROP_PREP);
                        autonIndex++;
                    }
                    break;
                case 10:
                    autonState = AutonState.SCORE_BACKDROP;
                    if (robot.outtake.articulation.equals(Outtake.Articulation.BACKDROP)) {
                        autonIndex++;
                    }
                    break;
                case 11:
                    autonState = AutonState.SCORE_DRIVE;
//                    if (robot.outtake.articulation.equals(Outtake.Articulation.TRAVEL_FROM_BACKDROP)) {
                        Sensors.distanceSensorsEnabled = true;
                        futureTimer = futureTime(1);
                        autonIndex++;
//                    }
                    break;
                case 12:
                    autonState = AutonState.SCORE_DRIVE;
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.2, 0), 0));
                    if (robot.sensors.leftDistSensorValue < SCORE_DRIVE_DISTANCE || robot.sensors.rightDistSensorValue < SCORE_DRIVE_DISTANCE|| isPast(futureTimer)) {
                        robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        robot.articulate(Robot.Articulation.TRAVEL_FROM_BACKDROP);
                        Sensors.distanceSensorsEnabled = false;
                        if(!Constants.runPixelStack){
                            strafeToParkBuild();
                            autonIndex = 21;
                        }
                        else {
                            driveToPixelStack1Build();
                            MecanumDrive.PARAMS.maxWheelVel = 25;
                            futureTimer = futureTime(1);
                            autonIndex++;
                        }
                    }
                    break;
                case 13:
                    autonState = AutonState.DRIVE_TO_PIXEL_STACK;
                    if (isPast(futureTimer))
                        if (!driveToPixelStack1.run(packet)) {
                            driveToPixelStack2Build();
                            futureTimer = futureTime(1);
                            autonIndex++;
                        }
                    break;
                case 14:
                    autonState = AutonState.IMU_TURN;
                    if (robot.driveTrain.turnUntilDegreesIMU(STANDARD_HEADING, turnToSpeed) || isPast(futureTimer)) {
                        robot.driveTrain.setPose(new Pose2d(new Vector2d(robot.driveTrain.pose.position.x, robot.driveTrain.pose.position.y), Math.toRadians(robot.sensors.driveIMUYaw)));
                        MecanumDrive.PARAMS.maxWheelVel = 50;
                        autonIndex++;
                    }
                    break;
                case 15:
                    autonState = AutonState.DRIVE_TO_PIXEL_STACK;
                    if (isPast(futureTimer))
                        if (!driveToPixelStack2.run(packet)) {
                            robot.intake.setIngestPixelHeight(4);
                            robot.articulate(Robot.Articulation.INGEST);
                            futureTimer = futureTime(2);
                            autonIndex++;
                        }
                    break;
                case 16:
                    autonState = AutonState.GET_FROM_PIXEL_STACK;
                    if (isPast(futureTimer)) {
                        robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        robot.intake.articulate(Intake.Articulation.SWALLOW);
                        approachBackdropBuild();
                        autonIndex++;
                    }
                    break;
                case 17:
                    autonState = AutonState.TRAVEL_BACKDROP;
                    if (!approachBackdrop.run(packet)) {
                        robot.articulate(Robot.Articulation.BACKDROP_PREP);
                        autonIndex++;
                    }
                    break;
                case 18:
                    autonState = AutonState.SCORE_BACKDROP;
                    if (robot.outtake.articulation.equals(Outtake.Articulation.BACKDROP)) {
                        autonIndex++;
                    }
                    break;
                case 19:
                    autonState = AutonState.SCORE_DRIVE;
                    if (robot.outtake.articulation.equals(Outtake.Articulation.TRAVEL)) {
                        Sensors.distanceSensorsEnabled = true;
                        futureTimer = futureTime(1);
                        autonIndex++;
                    }
                    break;
                case 20:
                    autonState = AutonState.SCORE_DRIVE;
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.2, 0), 0));
                    if (robot.sensors.leftDistSensorValue < SCORE_DRIVE_DISTANCE || robot.sensors.rightDistSensorValue < SCORE_DRIVE_DISTANCE || isPast(futureTimer)) {
                        robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        robot.articulate(Robot.Articulation.TRAVEL_FROM_BACKDROP);
                        Sensors.distanceSensorsEnabled = false;
                        strafeToParkBuild();
                        futureTimer = futureTime(1);
                        autonIndex++;
                    }
                    break;
                case 21:
                    autonState = AutonState.PARK;
                    if (!strafeToPark.run(packet)) {
                        autonIndex++;
                    }
                    break;
                case 22:
                    autonState = AutonState.DONE;
                    robot.positionCache.update(new CSPosition(robot.driveTrain.pose, robot.skyhook.getSkyhookLeftTicksCurrent(), robot.skyhook.getSkyhookRightTicksCurrent()), true);
                    return true;

            }
        dashboard.sendTelemetryPacket(packet);
        return false;
    }

}
