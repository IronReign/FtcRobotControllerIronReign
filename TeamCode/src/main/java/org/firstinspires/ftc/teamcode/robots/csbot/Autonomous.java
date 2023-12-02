package org.firstinspires.ftc.teamcode.robots.csbot;

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
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.LinkedHashMap;
import java.util.Map;


@Config (value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {

    public VisionProvider visionProvider;
    private Robot robot;
    private HardwareMap hardwareMap;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("autonIndex", autonIndex);
        telemetryMap.put("targetIndex", targetIndex);
        telemetryMap.put("stageoneposition", blueStageOnePosition == null? "null" : blueStageOnePosition.position.y);
        telemetryMap.put("deltaposition", (blueStageOnePosition == null || robot.driveTrain.pose == null)? "null" : robot.driveTrain.pose.position.y - blueStageOnePosition.position.y);
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

    Pose2d blueStageOnePosition;
    Pose2d blueStageTwoPosition;

    public static double STAGE_ONE_Y_COORDINATE = .5;

    public static double STAGE_TWO_Y_COORDINATE = -2.5;
    public static double STAGE_TWO_HEADING = 45;

    public static double STAGE_ONE_HEADING = 90;

    public void build() {
        autonIndex = 0;
        futureTimer = 0;
        targetIndex = visionProvider.getMostFrequentPosition().getIndex() + 1;

        Pose2d pose = startingPosition.getPose();

        blueStageOnePosition = P2D( pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE , STAGE_ONE_HEADING);
        blueStageTwoPosition = P2D(STAGE_TWO_Y_COORDINATE, blueStageOnePosition.position.y, STAGE_TWO_HEADING);

        redLeftStageOne = new SequentialAction(
                robot.driveTrain.actionBuilder(robot.driveTrain.pose)
                        .lineToYLinearHeading(blueStageOnePosition.position.y, blueStageOnePosition.heading)
                        .build()
        );
        redLeftStageTwo = new SequentialAction(
                robot.driveTrain.actionBuilder(blueStageOnePosition)
                        .lineToXLinearHeading(blueStageTwoPosition.position.x, blueStageTwoPosition.heading)
                        .build()
        );

//        redLeftStageTwo = new SequentialAction(
//                robot.driveTrain.actionBuilder(new )
//
//        )


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
                if(!redLeftStageOne.run(packet)) {
                    robot.intake.setAngleControllerTicks(Intake.BEATER_BAR_EJECT_ANGLE);
                    autonIndex++;
                }
            break;
            case 2:
                if(!redLeftStageTwo.run(packet)) {
                    futureTimer = futureTime(EJECT_WAIT_TIME);
                    autonIndex ++;
                }
                break;
            case 3:
                robot.intake.ejectBeaterBar();
                if(isPast(futureTimer))
                {
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
