package org.firstinspires.ftc.teamcode.robots.deepthought;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.field;
import static org.firstinspires.ftc.teamcode.robots.deepthought.field.Field.P2D;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.deepthought.field.POI;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.Sampler;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DTPosition;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.TelemetryProvider;

import java.util.LinkedHashMap;
import java.util.Map;


@Config(value = "AA_ITD_Auto_Basket")
public class Autonomous implements TelemetryProvider {
    public static double AUTON_OUTTAKE_WAIT_TIMER = 2;
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
        telemetryMap.put("field finalized?\t", field.finalized);
        telemetryMap.put("target basket\t", "" + field.basket.x + field.basket.y);
        telemetryMap.put("autonIndex\t", autonIndex);
        telemetryMap.put("outtakeIndex\t", autonOuttakeIndex);
        telemetryMap.put("outtake timer\t", isPast(autonOuttakeTimer));
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

    public Autonomous(Robot robot) {
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
        switch (autonIndex) {
            case 0:
                gameTimer = futureTime(27);
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
                if (autonSamplerOuttake(field.basket, packet)) {
                    robot.resetStates();
                    robot.articulate(Robot.Articulation.MANUAL);
//                    return true;
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
                if (autonSamplerIntake(field.ground2, packet)) {
                    if (robot.trident.sampler.servoPower == 0)
                        autonIndex++;
                    else {
                        autonIndex += 2;
                    }
//                    return true;
                }
                break;

            case 5:
                if (autonSamplerOuttake(field.basket, packet)) {
                    robot.resetStates();
                    robot.trident.sampler.articulate(Sampler.Articulation.MANUAL);

                    autonIndex++;
                }
                break;
            case 6:

                if (autonSamplerIntake(field.ground1, packet)) {
                    if (robot.trident.sampler.servoPower == 0)
                        autonIndex++;
                    else {
                        autonIndex += 2;
                    }
//                    return true;
                }
                break;
            case 7:
                if (autonSamplerOuttake(field.basket, packet)) {
                    robot.resetStates();
                    robot.trident.sampler.articulate(Sampler.Articulation.MANUAL);

                    autonIndex++;
                }
                break;

            case 8:
                if (autonSamplerIntake(field.ground3, packet)) {
                    if (robot.trident.sampler.servoPower == 0)
                        autonIndex++;
                    else {
                        autonIndex += 2;
                    }
//                    return true;
                }
                break;
            case 9:
                if (autonSamplerOuttake(field.basket, packet)) {
                    robot.resetStates();
                    robot.trident.sampler.articulate(Sampler.Articulation.MANUAL);

                    autonIndex++;
                }
                break;

            case 10:
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

    public boolean autonSamplerOuttake(POI basket, TelemetryPacket packet) {
        switch (autonOuttakeIndex) {
            case 0:
                robot.limelight.pipelineSwitch(2);
                robot.panTargetPosition = Robot.PAN_BASKET_APRILTAG;
                robot.resetStates();
                autonOuttakeTimer = futureTime(AUTON_OUTTAKE_WAIT_TIMER);
                autonOuttakeIndex++;
                break;
            case 1:
                if (isPast(autonOuttakeTimer)) {
                    Trident.enforceSlideLimits = false;
                    robot.articulate(Robot.Articulation.SAMPLER_OUTTAKE);
                    autonOuttakeTimer = futureTime(2);
                }
                if (robot.driveTrain.strafeToPose(basket.getPose(), packet) && robot.trident.sampler.slide.getCurrentPosition() > 400) {
                    autonOuttakeIndex++;
                }
                break;
            case 2:
                if (isPast(autonOuttakeTimer)) {
                    robot.aprilTagRelocalization();
                    robot.trident.sampler.adjustSlide(250);
                    robot.trident.sampler.servoPower = .3;
                    autonOuttakeTimer = futureTime(1);
                    autonOuttakeIndex++;
                }
                break;

            case 3:
//                robot.aprilTagRelocalization();
                if (isPast(autonOuttakeTimer)) {
                    robot.resetStates();
                    robot.articulate(Robot.Articulation.MANUAL);
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
    public long autonIntakeTimer = 0;
    int numAttempts = 2;

    public boolean autonSamplerIntake(POI ground, TelemetryPacket packet) {
        switch (autonIntakeIndex) {
            case 0:
                Robot.panTargetPosition = Robot.PAN_FORWARD;
                robot.limelight.pipelineSwitch(3);
                robot.articulate(Robot.Articulation.SAMPLER_PREP);
//                if (robot.driveTrain.strafeToPose(field.basketPrep.getPose(), packet)) {
                autonIntakeIndex++;

//                    robot.resetStates();
                //ADD LIMELIGHT ALIGNMENT HERE
//                }
            case 1:
                if (robot.driveTrain.strafeToPose(ground.getPose(), packet)) {
                    autonIntakeIndex++;
                    robot.resetStates();
                    autonIntakeTimer = futureTime(0);
                    //ADD LIMELIGHT ALIGNMENT HERE
                }
                break;
            case 2:
                if (isPast(autonIntakeTimer)) {
                    autonIntakeIndex++;
                }
                break;
            case 3:
//                if (ground.name.equals("GROUND3")) {
//                    autonIntakeIndex++;
//                } else {
//                    if (robot.alignOnSample()) {
                        autonIntakeIndex++;
//                    }
//                }
                break;
            case 4:

                if (robot.alignOnSample()) {
                    autonIntakeIndex++;
                }

                break;
            case 5:
                robot.articulate(Robot.Articulation.SAMPLER_INTAKE);
                if (robot.articulation == Robot.Articulation.MANUAL) {
                    autonIntakeIndex = 0;
                    return true;
                }
//                robot.articulate(Robot.Articulation.TRAVEL);


        }

        return false;
    }

    public static int pingPongIndex = 0;

    public void pingPong(TelemetryPacket packet) {
        robot.articulate(Robot.Articulation.SAMPLER_PREP);
        switch (pingPongIndex){
            case 0:
                if(robot.alignOnSample()) {
                    pingPongIndex++;
                }
                break;
            case 1:

                break;
            case 2:

                break;
        }

    }
}
