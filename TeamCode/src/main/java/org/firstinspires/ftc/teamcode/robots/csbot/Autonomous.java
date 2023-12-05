package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
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

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("autonIndex", autonIndex);
        telemetryMap.put("targetIndex", targetIndex);
        telemetryMap.put("stageoneposition", blueRightStageOnePosition == null ? "null" : blueRightStageOnePosition.position.y);
        telemetryMap.put("deltaposition", (blueRightStageOnePosition == null || robot.driveTrain.pose == null) ? "null" : robot.driveTrain.pose.position.y - blueRightStageOnePosition.position.y);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTONOMOUS";
    }

    // autonomous routines

    public static int targetIndex = 1;

    private Action
            redLeftStageOne, redLeftStageTwo,
            blueLeftStageOne, blueLeftStageTwo,
            blueRightStageOne, blueRightStageTwo,
            redRightStageOne, redRightStageTwo;

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

    public static double indexHeadingOffset = 0;

    //BASED ON BLUE_RIGHT_START
    public static double STAGE_ONE_Y_COORDINATE = .5;

    public static double STAGE_TWO_X_COORDINATE = -2.5;
    public static double STAGE_TWO_HEADING = 45;
    public static double BACKSTAGE_X_POSITION_OFFSET = 2.5;

    public void build() {
        autonIndex = 0;
        futureTimer = 0;

        Pose2d pose = startingPosition.getPose();

        blueRightStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, pose.heading.log());
        blueRightStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y, STAGE_TWO_HEADING - indexHeadingOffset);

        blueLeftStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, pose.heading.log());
        blueLeftStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE + BACKSTAGE_X_POSITION_OFFSET, blueRightStageOnePosition.position.y, STAGE_TWO_HEADING + 90 - indexHeadingOffset);

        redLeftStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, -STAGE_ONE_Y_COORDINATE, pose.heading.log());
        redLeftStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y, -STAGE_TWO_HEADING + indexHeadingOffset);

        redRightStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, -STAGE_ONE_Y_COORDINATE, pose.heading.log());
        redRightStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE + BACKSTAGE_X_POSITION_OFFSET, blueRightStageOnePosition.position.y, -STAGE_TWO_HEADING - 90 + indexHeadingOffset);

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
    }

    public void pickAutonToRun() {
        targetIndex = visionProvider.getMostFrequentPosition().getIndex() + 1;
        indexHeadingOffset = (targetIndex - 1) * 10;
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
        build();
    }


    public static int autonIndex;
    public static long futureTimer;
    public static int EJECT_WAIT_TIME = 4;


    public void execute(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();

            switch (autonIndex) {
                case 0:
                    autonIndex++;
                    break;
                case 1:
                    if (!stageOneToRun.run(packet)) {
                        robot.intake.setAngleControllerTicks(Intake.BEATER_BAR_EJECT_ANGLE);
                        autonIndex++;
                    }
                    break;
                case 2:
                    if (!stageTwoToRun.run(packet)) {
                        futureTimer = futureTime(EJECT_WAIT_TIME);
                        autonIndex++;
                    }
                    break;
                case 3:
                    robot.intake.ejectBeaterBar();
                    if (isPast(futureTimer)) {
                        robot.intake.beaterBarOff();
                        autonIndex++;
                    }
                    break;
                case 4:
                    break;

            }
        dashboard.sendTelemetryPacket(packet);
    }

}
