package org.firstinspires.ftc.teamcode.robots.deepthought;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
    public enum AutonState {
        INIT,
        DRIVE_TO_BASKET,
        OUTTAKE_TO_BASKET,
        DRIVE_TO_SUB,

    }


    public AutonState autonState = AutonState.INIT;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("autonState\t ", autonState);
        telemetryMap.put("auton index\t", autonIndex);
        telemetryMap.put("selectedPath\t", selectedPath);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTONOMOUS";
    }

    // autonomous routines

    public static int targetAprilTagIndex = 1;
    public static int selectedPath;


    public static double FIELD_INCHES_PER_GRID = 23.5;
    public static double AUTON_START_DELAY = 0;


//    Pose2d aprilTagApproachPosition;


//    private Action driveToPurplePixel;

    // misc. routines
    public StateMachine square, turn;

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
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
            switch (autonIndex) {
                case 0:
                    autonState = AutonState.INIT;
                    robot.positionCache.update(new DTPosition(robot.driveTrain.pose), true);
                    autonTimer = futureTime(AUTON_START_DELAY);
                    autonIndex++;
                    break;
                case 1:
                    if (System.nanoTime() > autonTimer) {
                        autonState = AutonState.DRIVE_TO_BASKET;
                        autonIndex++;
                    }
                    break;
                case 2:
                    if(robot.driveTrain.strafeToPose(Field.basket) {
                        autonState = AutonState.OUTTAKE_TO_BASKET;
                        autonIndex++;
                    }
                    break;
                case 3:
                    break;
                case 4:
                    break;
            }
        return false;


}
