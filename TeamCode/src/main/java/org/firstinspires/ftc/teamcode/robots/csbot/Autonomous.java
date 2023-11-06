package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.auton;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.FIELD_INCHES_PER_GRID;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.Position;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.statemachine.SingleState;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.slf4j.helpers.Util;

import java.util.LinkedHashMap;
import java.util.Map;

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
        telemetryMap.put("flag 1", !robot.driveTrain.trajectorySequenceRunner.isBusy());
        telemetryMap.put("flag 2", robot.visionProviderFinalized);
        telemetryMap.put("auton state", autonState.name() );
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

    public void build(Position startingPosition) {

        TrajectorySequence redRightAutonSequenceStageOne =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(FIELD_INCHES_PER_GRID)
                        .turn(90)
                        .build();


        redRight  = Utils.getStateMachine(new Stage())
                .addSingleState(trajectorySequenceToSingleState(redRightAutonSequenceStageOne))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .addSingleState(() -> {robot.visionProviderFinalized = false; robot.switchVisionProviders(); robot.createVisionProvider(); robot.visionProviderBack.initializeVision(hardwareMap); robot.visionProviderFinalized = true;})
                .addState(() -> robot.visionProviderFinalized)
                .addSingleState(() -> {robot.driveTrain.setWeightedDrivePower(new Pose2d(0, 1, 0)); autonState = AutonState.SCAN_FOR_APRILTAG;})

                .addSingleState(() -> {autonState = AutonState.TRAVEL;})
                .build();



        TrajectorySequence blueRightAutonSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(30*2)
                        .turn(90)
                        .back(42*2)
                        .turn(90)
                        .back(12*2)
                        .turn(-90)
                        .back(12*2)
                        .build();
        blueRight = trajectorySequenceToStateMachine(blueRightAutonSequence);

        TrajectorySequence redLeftAutonSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(30*2)
                        .turn(-90)
                        .back(42*2)
                        .turn(-90)
                        .back(12*2)
                        .turn(90)
                        .back(12*2)
                        .build();
        redLeft = trajectorySequenceToStateMachine(redLeftAutonSequence);

        TrajectorySequence blueLeftAutonSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(12*2)
                        .turn(-90)
                        .back(18*2)
                        .build();
        blueLeft = trajectorySequenceToStateMachine(blueLeftAutonSequence);

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


        //----------------------------------------------------------------------------------------------
        // Spline Routines
        //----------------------------------------------------------------------------------------------

    }

}
