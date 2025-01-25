package org.firstinspires.ftc.teamcode.robots.deepthought;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.field;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.deepthought.field.POI;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DTPosition;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.TelemetryProvider;

import java.util.LinkedHashMap;
import java.util.Map;


@Config(value = "AA_CS_Auton")
public class Autonomous implements TelemetryProvider {
    public static int numCycles = 4;
    private Robot robot;
    private HardwareMap hardwareMap;

    //
    public enum AutonState {
        INIT, DRIVE_TO_BASKET, OUTTAKE_TO_BASKET, DRIVE_TO_SUB,

    }


    public AutonState autonState = AutonState.INIT;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("autonState\t ", autonState);
        telemetryMap.put("autonIndex\t", autonIndex);
        telemetryMap.put("outtakeIndex\t", autonOuttakeIndex);
        telemetryMap.put("intakeIndex\t", autonIntakeIndex);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "AUTONOMOUS";
    }

    // autonomous routines

    public static int selectedPath;
    public int allianceMultiplier = 1;


    public static double FIELD_INCHES_PER_GRID = 23.5;
    public static double AUTON_START_DELAY = 0;

    public Autonomous(Robot r) {
        robot = r;
        this.hardwareMap = robot.hardwareMap;
        autonIndex = 0;
    }

    public static int autonIndex;
    public long autonTimer = futureTime(10);

    public boolean execute(TelemetryPacket packet) {
        if (!alliance.isRed()) {
            allianceMultiplier = -1;
        }
        robot.positionCache.update(new DTPosition(robot.driveTrain.getPose(), robot.trident.getShoulderCurrentPosition(), robot.trident.sampler.slide.getCurrentPosition(), robot.trident.speciMiner.slide.getCurrentPosition()), true);
        switch (autonIndex) {
            case 0:
                autonState = AutonState.INIT;
                autonTimer = futureTime(AUTON_START_DELAY);
                autonIndex++;
                break;
            case 1:
                if (isPast(autonTimer)) {
                    autonState = AutonState.DRIVE_TO_BASKET;
                    autonIndex++;
                }
                break;
            case 2:
                if(autonSamplerOuttake(packet)) {
                    robot.articulate(Robot.Articulation.TRAVEL);
                    autonIndex++;
                }
            case 3:
                if (robot.articulation == Robot.Articulation.MANUAL) {
                    autonIndex ++;
                }
                break;
            case 4:

                break;
            case 8:
                if (robot.articulation == Robot.Articulation.MANUAL && robot.trident.articulation == Trident.Articulation.MANUAL) {
                    robot.articulate(Robot.Articulation.TRAVEL);
                    autonIndex++;
                }
                break;
            case 9:
                robot.articulate(Robot.Articulation.TRAVEL);
                autonTimer = futureTime(2);
                autonIndex++;

                break;
            case 10:
                if (robot.driveTrain.strafeToPose(field.basket.getPose(), packet)) {
                    autonTimer = futureTime(.5);
                    autonIndex++;
                }
                if (isPast(autonTimer)) {
                    robot.trident.sampler.outtakeIndex = 0;
                    Trident.enforceSlideLimits = false;
                    robot.articulate(Robot.Articulation.SAMPLER_OUTTAKE);
                }
                break;

            case 11:
                if (robot.articulation.equals(Robot.Articulation.MANUAL) && isPast(autonTimer)) {
                    Trident.beaterPower = .5;
                    autonTimer = futureTime(3);
                    autonIndex++;
                }
                break;

            case 12:
                autonIndex++;
                break;

            case 13:
                if (isPast(autonTimer)) {
                    Trident.beaterPower = 0;
                    autonIndex++;
                }
                break;

            case 14:
                if (robot.driveTrain.strafeToPose(field.ground2.getPose(), packet)) {
                    robot.trident.beaterPower = 0;
                    robot.trident.sampler.intakeIndex = 0;
                    robot.articulate(Robot.Articulation.SAMPLER_INTAKE);
//                    autonTimer = futureTime(0);
                    autonIndex++;
                }
                break;
            case 15:
                if (isPast(autonTimer)) {
                    autonIndex++;
                }
                break;
            case 16:
                if (robot.articulation == Robot.Articulation.MANUAL && robot.trident.articulation == Trident.Articulation.MANUAL) {
                    robot.articulate(Robot.Articulation.TRAVEL);
                    autonIndex++;
                }
                break;
            case 17:
                robot.articulate(Robot.Articulation.TRAVEL);
                autonTimer = futureTime(2);
                autonIndex++;

                break;
            case 18:
                if (robot.driveTrain.strafeToPose(field.basket.getPose(), packet)) {
                    autonTimer = futureTime(.5);
                    autonIndex++;
                }
                if (isPast(autonTimer)) {
                    robot.trident.sampler.outtakeIndex = 0;
                    Trident.enforceSlideLimits = false;
                    robot.articulate(Robot.Articulation.SAMPLER_OUTTAKE);
                }
                break;

            case 19:
                if (robot.articulation.equals(Robot.Articulation.MANUAL) && isPast(autonTimer)) {
                    Trident.beaterPower = .5;
                    autonTimer = futureTime(3);
                    autonIndex++;
                }
                break;

            case 20:
                autonIndex++;
                break;

            case 21:
                if (isPast(autonTimer)) {
                    Trident.beaterPower = 0;

                    return true;
                }
                break;
        }
        return false;
    }

    //includes driving to outtake, actual dropoff, and leaves the robot in outtake position
    public int autonOuttakeIndex = 0;
    public int autonOuttakeTimer = 0;
    public boolean autonSamplerOuttake(TelemetryPacket packet) {
        switch (autonOuttakeIndex) {
            case 0:
                robot.resetStates();
                autonTimer = futureTime(2);
                autonOuttakeIndex ++;
                break;
            case 1:
                if (robot.driveTrain.strafeToPose(field.basket.getPose(), packet)) {
                    autonTimer = futureTime(.5);
                    autonIndex++;
                }
                if (isPast(autonTimer)) {
                    Trident.enforceSlideLimits = false;
                    robot.articulate(Robot.Articulation.SAMPLER_OUTTAKE);
                }
                break;
            case 2:
                if (robot.articulation.equals(Robot.Articulation.MANUAL) && isPast(autonTimer)) {
                   //TODO - START LIMELIGHT RELOCALIZATION HERE
                    robot.trident.sampler.servoPower = .5;
                    autonTimer = futureTime(2);
                    autonIndex++;
                }
                break;

            case 3:
                if(isPast(autonTimer)) {
                    robot.trident.sampler.servoPower = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    //includes driving to intake, 
    public int autonIntakeIndex = 0;
    public int autonIntakeTimer = 0;
    int numAttempts = 2;
    public boolean autonSamplerIntake(POI ground, TelemetryPacket packet) {
        switch (autonIntakeIndex){
            case 0:
                if(robot.driveTrain.strafeToPose(ground.getPose(), packet)) {
                    autonIntakeIndex++;
                    //ADD LIMELIGHT ALIGNMENT HERE
                }
                break;
            case 1:
                robot.articulate(Robot.Articulation.SAMPLER_INTAKE);
                if(robot.trident.sampler.sampleDetected()) {
                   autonIntakeIndex++;
                }
                break;
            case 2:

                break;
            case 3:
                break;
            case 4:
                break;
        }

        return false;
    }
}
