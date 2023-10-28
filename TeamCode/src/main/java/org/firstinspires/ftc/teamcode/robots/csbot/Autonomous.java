package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.Position;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Target;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config (value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {
    public VisionProvider visionProvider;
    private Robot robot;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("Can Stage", sixCanStage);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Autonomous";
    }

    // autonomous routines
    private StateMachine
            blueLeft, redRight, blueRight, redLeft;

    // misc. routines
    public StateMachine backAndForth, square, turn, lengthTest, diagonalTest, squareNoRR, Auton;

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.visionProvider = robot.visionProvider;
    }

    public StateMachine getStateMachine(Position startingPosition) {

                switch(startingPosition) {
                    case START_LEFT:
                        if(alliance.getMod())
                        return redLeft;
                        else return blueLeft;
                    case START_RIGHT:
                        if(alliance.getMod())
                            return redRight;
                        else return blueRight;
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

    public void build(Position startingPosition) {
        //----------------------------------------------------------------------------------------------
        // Misc. Routines
        //----------------------------------------------------------------------------------------------
        TrajectorySequence redRightAutonSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(12*2)
                        .turn(90)
                        .back(18*2)
                        .build();
        redRight = trajectorySequenceToStateMachine(redRightAutonSequence);

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

        switch (startingPosition) {
            case START_LEFT:

                break;
            case START_RIGHT:
                break;
        }
    }

    public int sixCanStage = 0;
}
