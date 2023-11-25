package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.FIELD_INCHES_PER_GRID;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.Position;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.P2D;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.csbot.rr_stuff.MecanumDrive;
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
        telemetryMap.put("autonIndex", autonIndex);
        telemetryMap.put("targetIndex", targetIndex);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTONOMOUS";
    }

    // autonomous routines

    public static int targetIndex = 1;

    private Action redLeftStageOne,redLeftStageTwo, redLeftStageThree;

    // misc. routines
    public StateMachine backAndForth, square, turn;

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        this.visionProvider = robot.visionProviderBack;
    }

    public static int STAGE_ONE_GRID_DISTANCE = 2;
    public static int STAGE_ONE_HEADING = -45;

    public void build() {
        autonIndex = 0;
        futureTimer = 0;
        targetIndex = visionProvider.getMostFrequentPosition().getIndex() + 1;

        Pose2d pose = robot.driveTrain.pose;

        Pose2d stageOnePosition = P2D(pose.position.y + (STAGE_ONE_GRID_DISTANCE * FIELD_INCHES_PER_GRID), pose.position.x, STAGE_ONE_HEADING);

//        Pose2d stageTwoPosition = stageOnePosition.plus(new Twist2d(new Vector2d()))

        redLeftStageOne = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .lineToYLinearHeading(stageOnePosition.position.y, stageOnePosition.heading)
                        .build());

//        redLeftStageTwo = new SequentialAction(
//                robot.driveTrain.actionBuilder(new )
//
//        )


    }


    public static int autonIndex;
    public static long futureTimer;
    public static int EJECT_WAIT_TIME;

    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (autonIndex) {
            case 0:
                autonIndex++;
                break;
            case 1:
                if(!redLeftStageOne.run(packet)) {
                    robot.intake.setAngleControllerTicks(Intake.BEATER_BAR_EJECT_ANGLE);
                    robot.intake.ejectBeaterBar();
                    futureTimer = futureTime(EJECT_WAIT_TIME);
                    autonIndex++;
                }
            break;
            case 2:
                if(isPast(futureTimer))
                {
                    robot.intake.beaterBarOff();
                    autonIndex++;
                }
                break;
            case 3:
                autonIndex++;
                break;
            case 4:
                break;

        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }




}
