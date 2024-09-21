package org.firstinspires.ftc.teamcode.robots.deepthought;

import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.DriveTrain.runTestPath;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DTPosition;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.LinkedHashMap;
import java.util.Map;


@Config(value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {

    public static double SCORE_DRIVE_DISTANCE = 3;
    private Robot robot;
    private HardwareMap hardwareMap;
//
//    public enum AutonState {
//        INIT,
//        TRAVEL_TO_PURPLE,
//        STRAFE,
//        SCORE_GROUND,
//        FIND_STANDARD_POSITION,
//        TRAVEL_BACKSTAGE,
//        DONE,
//        FIND_STANDARD_HEADING,
//        TRAVEL_BACKDROP,
//        ALIGN_WITH_APRILTAG,
//        FIND_APRIL_TAG_HEADING, PREP_FOR_PARK, PARK, SCORE_DRIVE, DRIVE_TO_PIXEL_STACK, GET_FROM_PIXEL_STACK, APRILTAG_STRAFE, IMU_TURN, APRILTAG_RELOCALIZE, SCORE_BACKDROP
//    }

//    public AutonState autonState = AutonState.INIT;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
//        telemetryMap.put("autonState", autonState);
        telemetryMap.put("auton index", autonIndex);
        telemetryMap.put("targetIndex", targetIndex);
        telemetryMap.put("targetAprilTag", targetAprilTagIndex);
        telemetryMap.put("selectedPath", selectedPath);
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
    public static int selectedPath;
    boolean testRunToWing = true;

    public static double FIELD_INCHES_PER_GRID = 23.5;
    public static double AUTON_START_DELAY = 0;

    double STANDARD_HEADING = 180;
//    Pose2d aprilTagApproachPosition;



    //values to actually use
    Pose2d[][] autonPaths;

//    private Action driveToPurplePixel;

    // misc. routines
    public StateMachine square, turn;

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        autonPaths = new Pose2d[7][13];
        autonIndex = 0;
    }

 // TODO - EXAMPLE LOCATION MARKER

//    public void driveToYellowPixelBuild() {
//        if (!Constants.driverSide) {
//            driveToYellowPixel = new SequentialAction(
//                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
//                            .setReversed(true)
//                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][6].heading.log()))
//                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
//                            .build()
//            );
//        } else {
//            driveToYellowPixel = new SequentialAction(
//                    robot.driveTrain.actionBuilder(robot.driveTrain.pose)
//                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
//                            .setReversed(true)
//                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][5].position), STANDARD_HEADING_RAD)
//                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][6].position), STANDARD_HEADING_RAD)
//                            .build()
//            );
//        }
//    }

    public static int autonIndex;
    public long autonTimer = futureTime(10);

    public boolean execute(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();
        if (runTestPath) {

        } else
            switch (autonIndex) {
                case 0:
                    robot.positionCache.update(new DTPosition(robot.driveTrain.pose), true);
                    return true;

            }
        dashboard.sendTelemetryPacket(packet);
        return false;
    }

}
