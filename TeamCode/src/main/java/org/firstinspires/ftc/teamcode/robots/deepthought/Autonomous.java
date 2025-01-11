package org.firstinspires.ftc.teamcode.robots.deepthought;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.field;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        telemetryMap.put("outtake state", robot.trident.outtakeIndex);
        telemetryMap.put("num cycles\t", numCycles);
        telemetryMap.put("selectedPath\t", selectedPath);
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

    public Autonomous(Robot robot) {
        this.robot = robot;
        this.hardwareMap = robot.hardwareMap;
        autonIndex = 0;
    }

    public static int autonIndex;
    public long autonTimer = futureTime(10);

    public boolean execute(TelemetryPacket packet) {
        if (!alliance.isRed()) {
            allianceMultiplier = -1;
        }
        switch (autonIndex) {
            case 0:
                autonState = AutonState.INIT;
                robot.positionCache.update(new DTPosition(robot.driveTrain.pose, robot.trident.shoulder.getCurrentPosition(), robot.trident.slide.getCurrentPosition()), true);
                autonTimer = futureTime(AUTON_START_DELAY);
                autonIndex++;
                break;
            case 1:
                if (isPast(autonTimer)) {
                    autonState = AutonState.DRIVE_TO_BASKET;
                    numCycles--;
                    autonTimer = futureTime(2);
                    autonIndex++;
                }
                break;
            case 2:
                if (robot.driveTrain.strafeToPose(field.basket.getPose(), packet)) {
                    autonTimer = futureTime(.5);
                    autonIndex++;
                }
                if(isPast(autonTimer)) {
                    robot.trident.outtakeIndex = 0;
                    Trident.enforceSlideLimits = false;
                    robot.articulate(Robot.Articulation.OUTTAKE);
                }
                break;
            case 3:
//                todo - test & tune preload score here
                if (robot.articulation.equals(Robot.Articulation.MANUAL) && isPast(autonTimer)) {
                    autonIndex++;
                }
                break;
            case 4:
                robot.trident.beaterPower = .8;
                autonTimer = futureTime(2);
                autonIndex++;
                break;
            case 5:
                Trident.colorSensorEnabled = true;
                if (isPast(autonTimer)) {
                    autonIndex++;
                }
                break;
            case 6:
                if (robot.driveTrain.strafeToPose(field.ground1.getPose(), packet)) {
                    robot.trident.beaterPower = 0;
                    Trident.intakeIndex = 0;
                    robot.articulate(Robot.Articulation.INTAKE);
//                    autonTimer = futureTime(0);
                    autonIndex++;
                }
                break;
            case 7:
//                if (isPast(autonTimer)) {
                    autonIndex++;
//                }
                break;
            case 8:
                if (robot.articulation == Robot.Articulation.MANUAL && robot.trident.articulation == Trident.Articulation.MANUAL) {
                    robot.articulate(Robot.Articulation.TRAVEL);
                    autonIndex++;
                }
                break;
            case 9:
                robot.articulate(Robot.Articulation.TRAVEL);
                autonTimer = futureTime(4);
//                robot.trident.slideTargetPosition -=50;
//                if(robot.trident.articulation == Trident.Articulation.MANUAL){
//                    robot.articulate(Robot.Articulation.TRAVEL);
//                    autonTimer = futureTime(3);
                    autonIndex++;
//                }
                break;
            case 10:
                if ((robot.driveTrain.strafeToPose(field.basketPrep.getPose(), packet) || isPast(autonTimer))) {;
//                    autonIndex ++;
                    return true;
                }
                break;
        }
        return false;
    }


}
