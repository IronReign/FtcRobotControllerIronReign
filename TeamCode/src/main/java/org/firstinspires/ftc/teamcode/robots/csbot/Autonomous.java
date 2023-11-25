package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.Position;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.P2D;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.statemachine.SingleState;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.LinkedHashMap;
import java.util.Map;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;


@Config (value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {
    public enum AutonState {
        INITIAL_DRIVEANDTURN,
        SCAN_FOR_APRILTAG,
        SCORE,
        TRAVEL

    }
    public VisionProvider visionProvider;
    public AutonState autonState = AutonState.INITIAL_DRIVEANDTURN;
    private Robot robot;
    private HardwareMap hardwareMap;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
//        telemetryMap.put("flag 1", !robot.driveTrain.trajectorySequenceRunner.isBusy());
        telemetryMap.put("flag 2", robot.visionProviderFinalized);
        telemetryMap.put("autonIndex", autonIndex);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTONOMOUS";
    }

    // autonomous routines
    private StateMachine
            blueLeft, redRight, blueRight, redLeft;

    // misc. routines
    public StateMachine backAndForth, square, turn;

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        this.visionProvider = robot.visionProviderBack;
    }

    public StateMachine getStateMachine(Position startingPosition) {

                switch(startingPosition) {
                    case START_LEFT_RED:
                        return redLeft;
                    case START_LEFT_BLUE:
                         return blueLeft;
                    case START_RIGHT_RED:
                            return redRight;
                    case START_RIGHT_BLUE:
                         return blueRight;
                    default:
                        throw new RuntimeException("no starting position given to getStateMachine()");
                }

    }


    public void build() {
        autonIndex = 0;
        futureTimer = 0;
    }

    public static int autonIndex;
    public static long futureTimer;

    public void execute() {

        switch (autonIndex) {
            case 0:
                futureTimer = futureTime(5);
                autonIndex++;
                break;
            case 1:
                if(robot.driveTrain.line())
                autonIndex++;
            break;
            case 2:
                robot.intake.setAngleControllerTicks(Intake.BEATER_BAR_WING_ANGLE + 90);
                robot.intake.ejectBeaterBar();
                if(isPast(futureTimer))
                {
                    autonIndex++;
                }
                break;
            case 3:
                robot.intake.beaterBarOff();
                autonIndex++;
                break;
            case 4:
                break;

        }


    }



//    TODO redo autonomous using the new mechanum drive code/drive train
/*
    private StateMachine trajectorySequenceToStateMachine(TrajectorySequence trajectorySequence) {
        return Utils.getStateMachine(new Stage())
                .addSingleState(() -> {
                    robot.driveTrain.followTrajectorySequenceAsync(
                            trajectorySequence
                    );
                })
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();
    }

    private SingleState trajectorySequenceToSingleState(TrajectorySequence trajectorySequence) {
        return () -> {
            robot.driveTrain.followTrajectorySequenceAsync(
                    trajectorySequence
            );
        };
    }



    public void build() {

        TrajectorySequence redRightAutonSequenceStageOne =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .lineToLinearHeading(P2D(3.5, -.5, 90))
                        .build();

        TrajectorySequence redLeftAutonSequenceStageOne =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .lineToLinearHeading(P2D(1.5, -.5, 90))
                        .build();

        TrajectorySequence redAutonSequencePathToMiddle =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .lineToLinearHeading(P2D(4.5, -.5, 180))
                        .build();
        TrajectorySequence redAutonPathToPixelStack =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .lineToLinearHeading(P2D(.5, -.5, 180))
                        .build();

        TrajectorySequence blueRightAutonSequenceStageOne =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .lineToLinearHeading(P2D(1.5, .5, -90))
                        .build();

        TrajectorySequence blueLeftAutonSequenceStageOne =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .lineToLinearHeading(P2D(3.5, .5, -90))
                        .build();

        TrajectorySequence blueAutonSequencePathToMiddle =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .lineToLinearHeading(P2D(4.5, .5, 180))
                        .build();
        TrajectorySequence blueAutonPathToPixelStack =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .lineToLinearHeading(P2D(.5, .5, 180))
                        .build();



        redRight  = Utils.getStateMachine(new Stage())
                //backs up to behind tape
                .addSingleState(trajectorySequenceToSingleState(redRightAutonSequenceStageOne))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //TODO - figure out how to score on correct tape here based on vision in init loop
                //inits AprilTag vision
                .addSingleState(() -> {robot.visionProviderFinalized = false; robot.switchVisionProviders(); robot.createVisionProvider(); robot.visionProviderBack.initializeVision(hardwareMap); robot.visionProviderFinalized = true;})
                .addState(() -> robot.visionProviderFinalized)
                //gets to in front of pixel board on the left
                .addSingleState(trajectorySequenceToSingleState(redAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //strafes slowly to scan for apriltag
                .addSingleState(() -> {robot.driveTrain.setWeightedDrivePower(new Pose2d(0, -.3, 0)); autonState = AutonState.SCAN_FOR_APRILTAG;})
                //TODO - STOP, ALIGN, SCORE, AND ADVANCE THE STATEMACHINE WHEN YOU SEE THE APRILTAG YOU WANT
                //this stops the robot, just to skip the above line during testing for only path
                .addSingleState(() -> robot.driveTrain.setWeightedDrivePower(new Pose2d(0, 0, 0)))
                .addState(() -> !robot.driveTrain.isBusy())
                //come back to the middle to get under the stage door
                .addSingleState(trajectorySequenceToSingleState(redAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //straight path to pixel stack closest to middle
                .addSingleState(trajectorySequenceToSingleState(redAutonPathToPixelStack))
                //TODO - INTAKE FROM PIXELSTACK HERE
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //come back up via stage door
                .addSingleState(trajectorySequenceToSingleState(redAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //TODO - repeat lines 136-153 for each time we want to cycle conestack
                .build();


        redLeft = Utils.getStateMachine(new Stage())
                //backs up to behind tape
                .addSingleState(trajectorySequenceToSingleState(redLeftAutonSequenceStageOne))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //TODO - figure out how to score on correct tape here based on vision in init loop
                //inits AprilTag vision
                .addSingleState(() -> {robot.visionProviderFinalized = false; robot.switchVisionProviders(); robot.createVisionProvider(); robot.visionProviderBack.initializeVision(hardwareMap); robot.visionProviderFinalized = true;})
                .addState(() -> robot.visionProviderFinalized)
                //gets to in front of pixel board on the left
                .addSingleState(trajectorySequenceToSingleState(redAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //strafes slowly to scan for apriltag
                .addSingleState(() -> {robot.driveTrain.setWeightedDrivePower(new Pose2d(0, -.3, 0)); autonState = AutonState.SCAN_FOR_APRILTAG;})
                //TODO - STOP, ALIGN, SCORE, AND ADVANCE THE STATEMACHINE WHEN YOU SEE THE APRILTAG YOU WANT
                //this stops the robot, just to skip the above line during testing for only path
                .addSingleState(() -> robot.driveTrain.setWeightedDrivePower(new Pose2d(0, 0, 0)))
                .addState(() -> !robot.driveTrain.isBusy())
                //come back to the middle to get under the stage door
                .addSingleState(trajectorySequenceToSingleState(redAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //straight path to pixel stack closest to middle
                .addSingleState(trajectorySequenceToSingleState(redAutonPathToPixelStack))
                //TODO - INTAKE FROM PIXELSTACK HERE
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //come back up via stage door
                .addSingleState(trajectorySequenceToSingleState(redAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //TODO - repeat lines 136-153 for each time we want to cycle conestack
                .build();



        blueRight = Utils.getStateMachine(new Stage())
                //backs up to behind tape
                .addSingleState(trajectorySequenceToSingleState(blueRightAutonSequenceStageOne))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //TODO - figure out how to score on correct tape here based on vision in init loop
                //inits AprilTag vision
                .addSingleState(() -> {robot.visionProviderFinalized = false; robot.switchVisionProviders(); robot.createVisionProvider(); robot.visionProviderBack.initializeVision(hardwareMap); robot.visionProviderFinalized = true;})
                .addState(() -> robot.visionProviderFinalized)
                //gets to in front of pixel board on the left
                .addSingleState(trajectorySequenceToSingleState(blueAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //strafes slowly to scan for apriltag
                .addSingleState(() -> {robot.driveTrain.setWeightedDrivePower(new Pose2d(0, .3, 0)); autonState = AutonState.SCAN_FOR_APRILTAG;})
                //TODO - STOP, ALIGN, SCORE, AND ADVANCE THE STATEMACHINE WHEN YOU SEE THE APRILTAG YOU WANT
                //this stops the robot, just to skip the above line during testing for only path
                .addSingleState(() -> robot.driveTrain.setWeightedDrivePower(new Pose2d(0, 0, 0)))
                .addState(() -> !robot.driveTrain.isBusy())
                //come back to the middle to get under the stage door
                .addSingleState(trajectorySequenceToSingleState(blueAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //straight path to pixel stack closest to middle
                .addSingleState(trajectorySequenceToSingleState(blueAutonPathToPixelStack))
                //TODO - INTAKE FROM PIXELSTACK HERE
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //come back up via stage door
                .addSingleState(trajectorySequenceToSingleState(blueAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //TODO - repeat lines 136-153 for each time we want to cycle conestack
                .build();


        blueLeft = Utils.getStateMachine(new Stage())
                //backs up to behind tape
                .addSingleState(trajectorySequenceToSingleState(blueLeftAutonSequenceStageOne))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //TODO - figure out how to score on correct tape here based on vision in init loop
                //inits AprilTag vision
                .addSingleState(() -> {robot.visionProviderFinalized = false; robot.switchVisionProviders(); robot.createVisionProvider(); robot.visionProviderBack.initializeVision(hardwareMap); robot.visionProviderFinalized = true;})
                .addState(() -> robot.visionProviderFinalized)
                //gets to in front of pixel board on the left
                .addSingleState(trajectorySequenceToSingleState(blueAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //strafes slowly to scan for apriltag
                .addSingleState(() -> {robot.driveTrain.setWeightedDrivePower(new Pose2d(0, -.3, 0)); autonState = AutonState.SCAN_FOR_APRILTAG;})
                //TODO - STOP, ALIGN, SCORE, AND ADVANCE THE STATEMACHINE WHEN YOU SEE THE APRILTAG YOU WANT
                //this stops the robot, just to skip the above line during testing for only path
                .addSingleState(() -> robot.driveTrain.setWeightedDrivePower(new Pose2d(0, 0, 0)))
                .addState(() -> !robot.driveTrain.isBusy())
                //come back to the middle to get under the stage door
                .addSingleState(trajectorySequenceToSingleState(blueAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //straight path to pixel stack closest to middle
                .addSingleState(trajectorySequenceToSingleState(blueAutonPathToPixelStack))
                //TODO - INTAKE FROM PIXELSTACK HERE
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //come back up via stage door
                .addSingleState(trajectorySequenceToSingleState(blueAutonSequencePathToMiddle))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                //TODO - repeat lines 136-153 for each time we want to cycle conestack
                .build();

        TrajectorySequence backAndForthSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(24)
                        .forward(24)
                        .build();
        backAndForth = trajectorySequenceToStateMachine(backAndForthSequence);

        TrajectorySequence squareSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .build();
        square = trajectorySequenceToStateMachine(squareSequence);

        TrajectorySequence turnSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .build();
        turn = trajectorySequenceToStateMachine(turnSequence);
    }
*/
}
