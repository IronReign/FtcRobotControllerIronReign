package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.DriveTrain.turnToSpeed;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake.FLIPPER_TRAVEL_ANGLE;
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

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.CSPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.AprilTagProvider;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.LinkedHashMap;
import java.util.Map;


@Config(value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {

    private static double APRIL_TAG_HEADING = 0;
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
        FIND_APRIL_TAG_HEADING, PREP_FOR_PARK, PARK, SCORE_DRIVE, DRIVE_TO_PIXEL_STACK, GET_FROM_PIXEL_STACK, SCORE_BACKDROP
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
        if(autonState == AutonState.ALIGN_WITH_APRILTAG) {
            telemetryMap.put("apriltag index", ((AprilTagProvider)robot.visionProviderBack).getIndex());
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
    public int targetAprilTagIndex = 1;
    public int visionProviderIndex;
    public double aprilTagOffset = .2;
    int allianceDirection = -1;
    public static int selectedPath;
    double STANDARD_HEADING_RAD = Math.PI;

    public static double FIELD_INCHES_PER_GRID = 23.5;

    double STANDARD_HEADING = 180;
    Pose2d aprilTagApproachPosition;
    Pose2d audienceIntermediate;
    Pose2d audienceIntermediateForward, audienceIntermediateDeep;
    Pose2d aprilTagAlign, aprilTagAlignClose, aprilTagAlignCrossed;


    //values to actually use
    Pose2d[][] autonPaths;

    private Action
            driveToPurplePixel,
            driveToYellowPixel,
            approachBackdrop,
            driveToPixelStack,
            sweep,
            strafeToPark
    ;

    // misc. routines
    public StateMachine backAndForth, square, turn;

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        autonPaths = new Pose2d[7][10];
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
        driveToYellowPixel = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .turnTo(switchSides(autonPaths[selectedPath][3].heading.log()))
                        .setReversed(true)
                        .splineTo(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                        .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                        .turnTo(STANDARD_HEADING_RAD)
                        .build()
        );
    }

    public void approachBackdropBuild() {
        approachBackdrop = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .setReversed(true)
                        .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                        .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                        .build()
        );
    }

    public void parkBuild() {
        strafeToPark = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .build()
        );
    }

    public void sweepBuild() {
        //we need to force the turn in one direction for these paths
        if(selectedPath == 5) {
            double intermediateAngle;
            if(alliance.getMod()) {
                intermediateAngle = Math.toDegrees(10);
            }
            else {
                intermediateAngle = Math.toDegrees(-10);
            }

            sweep = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .turnTo(robot.driveTrain.pose.heading.log() + intermediateAngle)
                            .turnTo(switchSides(autonPaths[selectedPath][2].heading.log()))
                            .build()
            );
        }
        //otherwise just one turn works
        else {
            sweep = new SequentialAction(
                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                            .turnTo(switchSides(autonPaths[selectedPath][2].heading.log()))
                            .build());
        }


    }

    public void driveToPurplePixelBuild() {
        driveToPurplePixel = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                    .setReversed(true)
                    .splineTo(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                        .build()
        );
    }

    public void driveToPixelStackBuild() {
        driveToPixelStack = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .setReversed(false)
                        .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                        .splineTo(switchSides(autonPaths[selectedPath][7].position), switchSides(autonPaths[selectedPath][7].heading.log()))
                        .build()
        );
    }

    public void strafeToParkBuild() {
        strafeToPark = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .strafeTo(new Vector2d(robot.driveTrain.pose.position.x, switchSides(autonPaths[selectedPath][5].position).y+20*allianceDirection*(selectedPath > 3? 1: -1)))
                        .build()
        );
    }

    public boolean strafeToAligned(int observedTag) {
        if(observedTag > targetAprilTagIndex) {
            robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0.3), 0));
            return false;
        }
        else if(observedTag < targetAprilTagIndex) {
            robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -0.3), 0));
            return false;
        }
        return true;
    }

    public void saveRandomizer(int visionIndex) {
        targetIndex = visionIndex + 1;
    }

    public void pickAutonToRun(Constants.Position startingPosition) {
        selectedPath = setPath(startingPosition, targetIndex);
    }

    public static Pose2d P2D(double x, double y, double deg) {
        return new Pose2d(x * FIELD_INCHES_PER_GRID, y * FIELD_INCHES_PER_GRID, Math.toRadians(deg));
    }

    public Pose2d switchSides(Pose2d p) {
        return new Pose2d(p.position.x, allianceDirection*p.position.y, -allianceDirection*p.heading.log());
    }
    public Vector2d switchSides(Vector2d v) {
        return new Vector2d(v.x, allianceDirection*v.y);
    }
    public double switchSides(double r) {
        return -allianceDirection*r;
    }

    public int setPath(Constants.Position startingPosition, int randomizer) { // 1, 2 or 3 for randomized prop
        autonIndex = 0;
        aprilTagApproachPosition = P2D(1.5,   1.5, STANDARD_HEADING);
        audienceIntermediate = P2D(1,.5,-10);
        audienceIntermediateForward = P2D(1, .5, STANDARD_HEADING);
        audienceIntermediateDeep = P2D(1.5,.5,-10);
        allianceDirection = startingPosition.getMod()? -1 : 1;
        targetAprilTagIndex = targetIndex + (startingPosition.getMod()? 3 : 0);
        //aprilTagAlign = new Pose2d (new Vector2d(switchSides(aprilTagApproachPosition.position).x,switchSides(aprilTagApproachPosition.position).y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x-1,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
        aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)), 0);
        aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x - 1,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)),
                0);
        aprilTagAlignCrossed = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)), Math.toRadians(-90));

        //assemble the paths
        autonPaths[1][1] = P2D(-2, .5, 90);
        autonPaths[1][2] = P2D(0, 0, -90);
        autonPaths[1][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[1][4] = audienceIntermediate;
        autonPaths[1][5] = aprilTagAlign;
        autonPaths[1][6] = audienceIntermediateForward;
        autonPaths[1][7] = P2D(-2.25, .5, STANDARD_HEADING);
        autonPaths[1][8] = audienceIntermediate;
        autonPaths[1][9] = aprilTagAlign;

        autonPaths[2][1] = P2D(-1.2, .43, 47);
        autonPaths[2][2] = P2D(0, 0, -90-35);
        autonPaths[2][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[2][4] = audienceIntermediate;
        autonPaths[2][5] = aprilTagAlign;
        autonPaths[2][6] = audienceIntermediateForward;
        autonPaths[2][7] = P2D(-2.25, .5, STANDARD_HEADING);
        autonPaths[2][8] = audienceIntermediate;
        autonPaths[2][9] = aprilTagAlign;
        System.out.println(switchSides(aprilTagAlign.position).y);

        autonPaths[3][1] = P2D(-1.7, 1, 90);
        autonPaths[3][2] = P2D(0, 0, -30);
        autonPaths[3][3] = P2D(0, 0, -130);
        autonPaths[3][4] = audienceIntermediateDeep;
        autonPaths[3][5] = aprilTagAlignCrossed;
        autonPaths[3][6] = audienceIntermediateForward;
        autonPaths[3][7] = P2D(-2.25, .5, STANDARD_HEADING);
        autonPaths[3][8] = audienceIntermediate;
        autonPaths[3][9] = aprilTagAlign;

        autonPaths[4][1] = P2D(startingPosition.getPose().position.x/FIELD_INCHES_PER_GRID+.3, 1.25, 90);
        autonPaths[4][2] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[4][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[4][4] = aprilTagAlignClose;
        autonPaths[4][5] = aprilTagAlign;
        autonPaths[4][6] = audienceIntermediateForward;
        autonPaths[4][7] = P2D(-2.25, .5, STANDARD_HEADING);
        autonPaths[4][8] = audienceIntermediate;
        autonPaths[4][9] = aprilTagAlign;

        autonPaths[5][1] = P2D(startingPosition.getPose().position.x/FIELD_INCHES_PER_GRID, 1.8, 90);
        autonPaths[5][2] = P2D(0, 0, 90);
        autonPaths[5][3] = P2D(0, 0, 170);
        autonPaths[5][4] = aprilTagAlignClose;
        autonPaths[5][5] = aprilTagAlign;
        autonPaths[5][6] = audienceIntermediateForward;
        autonPaths[5][7] = P2D(-2.25, .5, STANDARD_HEADING);
        autonPaths[5][8] = audienceIntermediate;
        autonPaths[5][9] = aprilTagAlign;

        autonPaths[6][1] = P2D(1.4, 41.2 / 23.5 , -15);
        autonPaths[6][2] = P2D(0, 0, 125);
        autonPaths[6][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[6][4] = aprilTagAlignClose;
        autonPaths[6][5] = aprilTagAlign;
        autonPaths[6][6] = audienceIntermediateForward;
        autonPaths[6][7] = P2D(-2.25, .5, STANDARD_HEADING);
        autonPaths[6][8] = audienceIntermediate;
        autonPaths[6][9] = aprilTagAlign;

        int rando = randomizer;
        if (allianceDirection==1 && randomizer==1) rando = 3;
        if (allianceDirection==1 && randomizer==3) rando = 1;
        return (startingPosition.equals(Constants.Position.START_RIGHT_RED)||startingPosition.equals(Constants.Position.START_LEFT_BLUE))?
                3+rando :rando;
    }

    public static int autonIndex;
    public static long futureTimer;

    int firstIndex = 0;



    public boolean execute(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();

            switch (autonIndex) {
                case 0:
                    futureTimer = futureTime(.4);
                    driveToPurplePixelBuild();
                    autonIndex++;
                    break;
                case 1:
                    autonState = AutonState.BACK_UP;
                    if (!driveToPurplePixel.run(packet)) {
                        robot.intake.articulate(Intake.Articulation.DOWN);
                        sweepBuild();
                        autonIndex++;
                    }
                    break;
                case 2:
                    autonState = AutonState.SCORE_GROUND;
                    if (!sweep.run(packet)) {
                        robot.intake.articulate(Intake.Articulation.EJECT);
                        autonIndex++;
                    }
                    break;
                case 3:
                    if (robot.intake.readyForTravel()) {
                        robot.outtake.setTargetAngle(FLIPPER_TRAVEL_ANGLE);
                        driveToYellowPixelBuild();
                        autonIndex++;
                    }
                    break;
                case 4:
//                    if(robot.outtake.ingestFromTravel()) //todo should call an articulation instead
//                        {
//                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), .2));
                    autonIndex++;
//                        }
                    break;
                case 5:
                    autonState = AutonState.TRAVEL_BACKDROP;
                    if (!driveToYellowPixel.run(packet)) {
                        autonIndex++;
                    }
                    break;

                case 6:
                    if(robot.driveTrain.turnUntilDegreesIMU(STANDARD_HEADING,turnToSpeed)) {
                        robot.driveTrain.pose = new Pose2d(new Vector2d(robot.driveTrain.pose.position.x, robot.driveTrain.pose.position.y), Math.toRadians(robot.driveTrain.imuAngle));
                        robot.switchVisionProviders();
                        autonIndex ++;
                    }
                    break;
                case 7:
                    autonState = AutonState.ALIGN_WITH_APRILTAG;
                    robot.enableVision();
                    futureTimer = futureTime(2);
                    autonIndex++;
                    break;
                case 8:
                    robot.enableVision();
                    if(isPast(futureTimer) && robot.getAprilTagDetections() != null) {
                        robot.aprilTagRelocalization(targetAprilTagIndex);
//                        robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//                        robot.switchVisionProviders();
//                        robot.articulate(Robot.Articulation.BACKDROP_PREP);
//                        autonIndex++;
                    }
                    break;
                case 9:
                    autonState = AutonState.SCORE_BACKDROP;
                    if(robot.outtake.articulation.equals(Outtake.Articulation.BACKDROP)) {
                        autonIndex++;
                        futureTimer = futureTime(.25);
                    }
                    break;
                case 10:
                    autonState = AutonState.SCORE_DRIVE;
                    if(robot.outtake.articulation.equals(Outtake.Articulation.TRAVEL)) {
                        autonIndex++;
                        futureTimer = futureTime(.25);
                    }
                    break;
                case 11:
                    autonState = AutonState.SCORE_DRIVE;
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.3, 0), 0));
                    if(robot.driveTrain.backDistanceSensorValue < 7) {
                        robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        robot.articulate(Robot.Articulation.TRAVEL);
                        robot.outtake.articulate(Outtake.Articulation.TRAVEL_FROM_BACKDROP);
                        driveToPixelStackBuild();
                        futureTimer = futureTime(1);
                        autonIndex++;
                    }
                    break;
                case 12:
                    autonState = AutonState.DRIVE_TO_PIXEL_STACK;
                    if(isPast(futureTimer))
//                    if (!driveToPixelStack.run(packet)) {
                        autonIndex++;
//                    }
                    break;
                case 13:
                    autonState = AutonState.GET_FROM_PIXEL_STACK;
//                    robot.intake.setIngestPixelHeight(4);
//                    robot.intake.articulate(Intake.Articulation.INGEST);
//                    futureTimer = futureTime(1);
//                    if(isPast(futureTimer)) {
//                        robot.intake.setIngestPixelHeight(3);
//                        futureTimer = futureTime(1);
                        autonIndex++;
//                    }
                    break;
                case 14:
//                    if(isPast(futureTimer)) {
//                          robot.intake.articulate(Intake.Articulation.SWALLOW);
//                          approachBackdropBuild();
                            autonIndex++;
//                    }
                    break;
                case 15:
                    autonState = AutonState.TRAVEL_BACKDROP;
//                    if (!approachBackdrop.run(packet)) {
                        autonIndex++;
//                    }
                    break;
                case 16:
                    autonState = AutonState.SCORE_BACKDROP;
//                    if(robot.outtake.articulation.equals(Outtake.Articulation.BACKDROP)) {
                        autonIndex++;
//                   }
                    break;
                case 17:
                    autonState = AutonState.SCORE_DRIVE;
//                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.3, 0), 0));
//                    if(robot.driveTrain.backDistanceSensorValue < 7) {
//                        robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//                        robot.articulate(Robot.Articulation.TRAVEL);
//                        robot.outtake.articulate(Outtake.Articulation.TRAVEL_FROM_BACKDROP);
//                        futureTimer = futureTime(1);
                        strafeToParkBuild();
                        autonIndex++;
//                    }
                    break;
                case 18:
                    autonState = AutonState.PARK;
                    if(!strafeToPark.run(packet)) {
                        autonIndex++;
                    }
                    break;
                case 19:
                    autonState = AutonState.DONE;
                    robot.positionCache.update(new CSPosition(robot.driveTrain.pose, robot.skyhook.getSkyhookLeftTicksCurrent(), robot.skyhook.getSkyhookRightTicksCurrent()), true);
                    return true;

            }
            dashboard.sendTelemetryPacket(packet);
            return false;
    }

}
