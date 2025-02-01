package org.firstinspires.ftc.teamcode.robots.deepthought;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.field;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.deepthought.field.POI;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.SpeciMiner;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DTPosition;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.TelemetryProvider;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "AA_ITD_Auto_Spec")
public class AutoSpecimens implements TelemetryProvider {
    public static double AUTON_WAIT_TIMER = 1;
    public static int numCycles = 4;
    private Robot robot;
    private HardwareMap hardwareMap;

    //
    public enum AutonState {
        INIT, DRIVE_TO_HIGHBAR, DRIVE_TO_OZONE, OUTTAKE_TO_HIGHBAR, DRIVE_TO_SUB,
    }

    public AutonState autonState = AutonState.INIT;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("autonState\t ", autonState);
        telemetryMap.put("autonIndex\t", autonIndex);
        telemetryMap.put("outtakeIndex\t", autonOuttakeIndex);
        telemetryMap.put("outtake timer\t", isPast(autonOuttakeTimer));
        telemetryMap.put("intakeIndex\t", autonIntakeIndex);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTOSPEC";
    }

    // autonomous routines

    public static int selectedPath;
    public int allianceMultiplier = 1;


    public static double FIELD_INCHES_PER_GRID = 23.5;
    public static double AUTON_START_DELAY = 0;

    public AutoSpecimens(Robot robot) {
        this.robot = robot;
        this.hardwareMap = this.robot.hardwareMap;
        autonIndex = 0;
    }

    public static int autonIndex;
    public long autonTimer = futureTime(10);
    public long gameTimer;

    public boolean execute(TelemetryPacket packet) {
        if (!alliance.isRed()) {
            allianceMultiplier = -1;
        }
        robot.positionCache.update(new DTPosition(robot.driveTrain.getPose(), robot.trident.getShoulderCurrentPosition(), robot.trident.sampler.slide.getCurrentPosition(), robot.trident.speciMiner.slide.getCurrentPosition()), false);
        switch (autonIndex) { //auton delay
            case 0:
                gameTimer = futureTime(27);
                autonState = AutonState.INIT;
                autonTimer = futureTime(AUTON_START_DELAY);
                autonIndex++;
                break;
            case 1: // go to hibar
                if (isPast(autonTimer)) {
                    autonState = AutonState.DRIVE_TO_HIGHBAR;
                    autonIndex++;
                }
                break;
            case 2:
                if (autonSpecimenOuttake(packet)) {
                    robot.resetStates();
                    robot.articulate(Robot.Articulation.MANUAL);
                    autonIndex++;
                }
                break;
            case 3:
                if (robot.articulation == Robot.Articulation.MANUAL) {
                    autonIndex++;
                }
                break;
            case 4:
                //todo - need to find a way to manage failed intakes
                if (autonSweepSample(field.ground2, packet)) {
                    autonIndex++;
                }
                break;

            case 5:
                if (autonSpecimenOuttake(packet)) {
                    robot.resetStates();
                    robot.trident.speciMiner.articulate(SpeciMiner.Articulation.MANUAL);

                    autonIndex = 7;
                }
                break;
            case 6:

                if (autonSweepSample(field.ground1, packet)) {
                    autonIndex++;
                }
                break;
            case 7:
                robot.articulate(Robot.Articulation.TRAVEL);
                robot.positionCache.update(new DTPosition(robot.driveTrain.getPose(), robot.trident.getShoulderCurrentPosition(), robot.trident.sampler.slide.getCurrentPosition(), robot.trident.speciMiner.slide.getCurrentPosition()), true);
                autonIndex = 0;
                return true;
        }
        return false;
    }

    //includes driving to outtake, actual dropoff, and leaves the robot in outtake position
    public int autonOuttakeIndex = 0;
    public long autonOuttakeTimer = 0;

    public boolean autonSpecimenOuttake(TelemetryPacket packet) {
        switch (autonOuttakeIndex) {
            case 0: // not sure we need another wait here if there is one in execute()
                robot.resetStates();
                autonOuttakeTimer = futureTime(AUTON_WAIT_TIMER);
                autonOuttakeIndex++;
                break;
            case 1: // score the preload alliance sample
                if (isPast(autonOuttakeTimer)) {
                    Trident.enforceSlideLimits = true;
                    robot.articulate(Robot.Articulation.SPECIMINER_OUTTAKE);
                    autonOuttakeTimer = futureTime(1.75);
                    //todo - set Speciminer and Shoulder for hibar prep while driving

                }
                // drive to hibar prep location
                if (robot.driveTrain.strafeToPose(field.hibarPrep.getPose(), packet)) {
                    autonOuttakeIndex++;
                }
                break;
            case 2: //todo keep modifying for specimens
                if (isPast(autonOuttakeTimer)) {
//                    robot.aprilTagRelocalization();
                    autonOuttakeTimer = futureTime(2);
                    autonOuttakeIndex++;
                }
                break;

            case 3:
                robot.aprilTagRelocalization();
                if (isPast(autonOuttakeTimer)) {
                    robot.trident.sampler.servoPower = 0;
                    autonOuttakeIndex = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    //includes driving to ground intake, intaking, ends in tuck()
    public int autonIntakeIndex = 0;
    public int autonIntakeTimer = 0;
    int numAttempts = 2;

    public boolean autonSweepSample(POI ground, TelemetryPacket packet) {
        switch (autonIntakeIndex) {
            case 0: // drive from hibar or from starting position to safe intermediate
                if (robot.driveTrain.strafeToPose(field.zig.getPose(), packet)) {
                    autonIntakeIndex++;
                    //robot.resetStates();
                }
            case 1: // get beyond alliance samples
                if (robot.driveTrain.strafeToPose(field.zag.getPose(), packet)) {
                    autonIntakeIndex++;
                    //robot.resetStates();
                }
                break;
            case 2: //drive to coral target sample
                if (robot.driveTrain.strafeToPose(ground.getPose(), packet)) {
                    autonIntakeIndex++;
                    //robot.resetStates();
                }
                break;
            case 3: // push to ozone
                if (robot.driveTrain.strafeToPose(new Pose2d(ground.getPose().position.x, -2.3, 90), packet)) {
                    autonIntakeIndex++;
                    //robot.resetStates();
                    robot.articulate(Robot.Articulation.TRAVEL);
                    autonIntakeIndex = 0;
                    return true;
                }
                break;

        }

        return false;
    }


}
