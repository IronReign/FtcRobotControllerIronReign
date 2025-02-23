package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants.FIELD_INCHES_PER_GRID;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Skyhook;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "CSAUTONAV")
public class AutoNav implements TelemetryProvider {
    Robot robot;
    Field field;
    public static boolean cycling;
    public static boolean intaking;
    public static boolean backdropSide = true;
    int preferredRoute = 5;
    int intakeIndex = 0;
    int outtakeIndex = 0;



    AutoNav(Robot robot, Field field) {
        this.robot = robot;
        this.field = field;
        //todo - assuming robot is cycling and preferredRoute = 3 for now, figure out how to pass it in cleanly
        this.cycling = true;
        this.intaking = true;
//        preferredRoute =
        clearStates();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Preferred Route\t", preferredRoute);
        telemetryMap.put("Cycling\t", cycling);
        telemetryMap.put("intaking\t", intaking);
        telemetryMap.put("intakeindex\t", intakeIndex);
        telemetryMap.put("outtakeindex", outtakeIndex);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {return "AUTO_NAV";}

    public void run(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();
        if(cycling) {
            if(intaking) {
                //assuming wing intake for now
                if (retrieve(packet, preferredRoute, true))
                    intaking = false;
            }
            else {
                //assuming middle apriltag for now
                if(deliver(packet, preferredRoute, 2))
                    intaking = true;
            }
        }
        dashboard.sendTelemetryPacket(packet);
    }

    public int setPreferredRoute(int preferredRoute) {
        this.preferredRoute = preferredRoute;
        return this.preferredRoute;
    }

    public void clearStates() {
        intakeIndex = 0;
        outtakeIndex = 0;
    }

    SequentialAction pathToRetrieval;
    public boolean retrieve(TelemetryPacket packet, int preferredRoute, boolean wing) {
        switch (intakeIndex) {
            case 0:
                //get poi for desired intake location, assuming wing for now
//                if(wing)
                //should be field.WING_INTAKE for correct flipping, this was used for half field testing
                POI poi = field.WING_INTAKE;
                //build path
                pathToRetrieval = field.pathToPOI(robot, poi, preferredRoute);
                intakeIndex++;
                break;
            case 1:
                if(!pathToRetrieval.run(packet)) {
                    //set intake to ingest and drive forward very slowly
                    robot.articulate(Robot.Articulation.INGEST);
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(.15, 0), 0));
                    intakeIndex ++;
                }
                break;
            case 2:
                //only happens after both pixel sensors are asserted and swallow is done
                if(robot.articulation.equals(Robot.Articulation.TRAVEL)) {
                    intakeIndex = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    long retrievalTimer = 0;
    public SequentialAction pathToDelivery;
    public SequentialAction aprilTagStrafe;
    public boolean deliver(TelemetryPacket packet, int preferredRoute, int targetIndex) {
        switch (outtakeIndex) {
            case 0:
                //select outtake score locations
//                POI poi = alliance.isRed()? field.scoreLocations.get(targetIndex + 3) : field.scoreLocations.get(targetIndex);
                POI poi = field.scoreLocations.get(2);
                //build path
                pathToDelivery = field.pathToPOI(robot, poi, preferredRoute);
                outtakeIndex++;
                break;
            case 1:
                robot.enableVision();
                if(!pathToDelivery.run(packet)) {
                    robot.aprilTagRelocalization(alliance.isRed()? 5 : 2);
                    aprilTagStrafe = field.pathToPOI(robot, field.scoreLocations.get(alliance.isRed()? 5 : 2), preferredRoute);
                    outtakeIndex ++;
                }
                break;
            case 2:
                if(!aprilTagStrafe.run(packet)){
                    //deploy outtake and drive backward very slowly
                    robot.articulate(Robot.Articulation.BACKDROP_PREP);
                    //backdrop prep just takes too long lmao
                    retrievalTimer = futureTime(3);
                    outtakeIndex ++;
                }
                break;
            case 3:
                if(isPast(retrievalTimer)) {
                    Robot.sensors.distanceSensorsEnabled = true;
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.15, 0), 0));
                    outtakeIndex++;
                }
                break;
            case 4:
                robot.distanceSensorX();
                //todo - change to a more robust exit condition, this assumes pixels have been scored when dist is small
                if(robot.sensors.leftDistSensorValue < 6) {
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-0, 0), 0));
                    Robot.sensors.distanceSensorsEnabled = false;
                    robot.articulate(Robot.Articulation.TRAVEL_FROM_BACKDROP);
                    outtakeIndex ++;
                }
                break;
            case 5:
                //our job's done here
                if(robot.articulation.equals(Robot.Articulation.TRAVEL)) {
                    outtakeIndex = 0;
                    return true;
                }
                break;
        }
        return false;
    }
    private Action adjustForPrepHang;
    private Action driveToHang;
    private Action driveToDrone;
    public void autoEndgameBuild() {
        ArrayList<SubZone> arr = field.getSubZones(robot.driveTrain.getPose());
        if(arr.contains(SubZone.BACKDROP)) {
            if(backdropSide) {
                driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .strafeToLinearHeading(new Vector2d(12, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                        .build();
                adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .build();
                driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                        .build();
            }
            else {
                driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .strafeToLinearHeading(new Vector2d(12, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                        .build();
                adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .strafeToLinearHeading(new Vector2d(-10, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                        .build();
                driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                        .build();
            }
        }
        else if(arr.contains(SubZone.WING)) {
            if(backdropSide) {
                driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .setReversed(true)
                        .strafeToLinearHeading(new Vector2d(-2 * FIELD_INCHES_PER_GRID, alliance.isRed() ? -.5 * FIELD_INCHES_PER_GRID : .5 * FIELD_INCHES_PER_GRID), Math.toRadians(180))
                        .strafeTo(new Vector2d(1 * FIELD_INCHES_PER_GRID, alliance.isRed() ? -.5 * FIELD_INCHES_PER_GRID : .5 * FIELD_INCHES_PER_GRID))
                        .setReversed(false)
                        .strafeTo(new Vector2d(12, alliance.isRed() ? -35 : 35))
                        .build();
                adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .build();
                driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                        .build();
            }
            else {
                driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .setReversed(true)
                        .strafeToLinearHeading(new Vector2d(-2 * FIELD_INCHES_PER_GRID, alliance.isRed() ? -.5 * FIELD_INCHES_PER_GRID : .5 * FIELD_INCHES_PER_GRID), Math.toRadians(180))
                        .strafeTo(new Vector2d(1 * FIELD_INCHES_PER_GRID, alliance.isRed() ? -.5 * FIELD_INCHES_PER_GRID : .5 * FIELD_INCHES_PER_GRID))
                        .setReversed(false)
                        .strafeTo(new Vector2d(12, alliance.isRed() ? -58.5 : 58.5))
                        .build();
                adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .strafeToLinearHeading(new Vector2d(-10, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                        .build();
                driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                        .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                        .build();
            }
        }
        else {
            if(field.getZone(robot.driveTrain.getPose()) == Field.Zone.AUDIENCE) {
                if(backdropSide) {
                    driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .setReversed(true)
                            .strafeTo(new Vector2d(-1.5 * FIELD_INCHES_PER_GRID, alliance.isRed() ? -2.5 * FIELD_INCHES_PER_GRID : 2.5 * FIELD_INCHES_PER_GRID))
                            .strafeTo(new Vector2d(1 * FIELD_INCHES_PER_GRID, alliance.isRed() ? -2.5 * FIELD_INCHES_PER_GRID : 2.5 * FIELD_INCHES_PER_GRID))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(12, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                            .build();
                    adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .build();
                    driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                            .build();
                }
                else {
                    driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .setReversed(true)
                            .strafeTo(new Vector2d(-1.5 * FIELD_INCHES_PER_GRID, alliance.isRed() ? -2.5 * FIELD_INCHES_PER_GRID : 2.5 * FIELD_INCHES_PER_GRID))
                            .strafeTo(new Vector2d(1 * FIELD_INCHES_PER_GRID, alliance.isRed() ? -2.5 * FIELD_INCHES_PER_GRID : 2.5 * FIELD_INCHES_PER_GRID))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(12, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                    adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-10, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                    driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                }
            }
            else if(field.getZone(robot.driveTrain.getPose()) == Field.Zone.RIGGING){
                if(backdropSide) {
                    driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .setReversed(true)
                            .strafeTo(new Vector2d(1 * FIELD_INCHES_PER_GRID, -.5 * FIELD_INCHES_PER_GRID))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(12, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                            .build();
                    adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .build();
                    driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                            .build();
                }
                else {
                    driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .setReversed(true)
                            .strafeTo(new Vector2d(1 * FIELD_INCHES_PER_GRID, -.5 * FIELD_INCHES_PER_GRID))
                            .setReversed(false)
                            .strafeToLinearHeading(new Vector2d(12, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                    adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-10, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                    driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                }
            }
            else {
                if(backdropSide) {
                    driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(12, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                            .build();
                    adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .build();
                    driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -35 : 35), Math.toRadians(180))
                            .build();
                }
                else {
                    driveToDrone = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(12, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                    adjustForPrepHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-10, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                    driveToHang = robot.driveTrain.actionBuilder(robot.driveTrain.getPose())
                            .strafeToLinearHeading(new Vector2d(-16, alliance.isRed() ? -58.5 : 58.5), Math.toRadians(180))
                            .build();
                }
            }
        }
    }

    private long autoEndgameTimer = 0;
    private int autoEndgameIndex = 0;
    public boolean autoEndgame() {
        switch (autoEndgameIndex) {
            case 0:
                autoEndgameBuild();
                break;
            case 1:
                if (!driveToDrone.run(new TelemetryPacket())) {
                    driveToDrone = null;
                    autoEndgameIndex++;
                }
                break;
            case 2:
                robot.articulate(Robot.Articulation.LAUNCH_DRONE);
                autoEndgameTimer = futureTime(2);
                autoEndgameIndex++;
                break;
            case 3:
                if (isPast(autoEndgameTimer) && !adjustForPrepHang.run(new TelemetryPacket())) {
                    robot.articulate(Robot.Articulation.PREP_FOR_HANG);
                    autoEndgameIndex++;
                }
                break;
            case 4:
                if(!driveToHang.run(new TelemetryPacket())) {
                    autoEndgameIndex++;
                }
                break;
            case 5:
                if (robot.skyhook.articulation.equals(Skyhook.Articulation.PREP_FOR_HANG)) {
                    robot.articulate(Robot.Articulation.HANG);
                    autoEndgameIndex = 0;
                    return true;
                }
        }
        return false;
    }
    public void assumeControl() {
        //todo - I can't really think of anything else that needs to be done?
        robot.enterTravel();
        clearStates();
    }

    public void relinquishControl() {
        //todo - I can't really think of anything else that needs to be done?
        Robot.sensors.distanceSensorsEnabled = false;
        clearStates();
    }
}
