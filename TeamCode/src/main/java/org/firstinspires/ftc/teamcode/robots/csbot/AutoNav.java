package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;

public class AutoNav {
    Robot robot;
    Field field;
    public static boolean cycling;
    public static boolean intaking;
    int preferredRoute = 3;

    AutoNav(Robot robot, Field field) {
        this.robot = robot;
        this.field = field;
        //todo - assuming robot is cycling and preferredRoute = 3 for now, figure out how to pass it in cleanly
        this.cycling = true;
        this.intaking = true;
//        preferredRoute =
        clearStates();
    }

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
    int intakeIndex = 0;
    public boolean retrieve(TelemetryPacket packet, int preferredRoute, boolean wing) {
        switch (intakeIndex) {
            case 0:
                //get poi for desired intake location, assuming wing for now
//                if(wing)
                POI poi = field.WING_INTAKE;
                //build path
                pathToRetrieval = field.pathToPOI(robot, poi, preferredRoute);
                intakeIndex++;
                break;
            case 1:
                if(pathToRetrieval.run(packet)) {
                    //set intake to ingest and drive forward very slowly
                    robot.articulate(Robot.Articulation.INGEST);
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(.1, 0), 0));
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

    public SequentialAction pathToDelivery;
    int outtakeIndex = 0;
    public boolean deliver(TelemetryPacket packet, int preferredRoute, int targetIndex) {
        switch (outtakeIndex) {
            case 0:
                //select outtake score locations
                POI poi = alliance.getMod()? field.scoreLocations.get(targetIndex + 3) : field.scoreLocations.get(targetIndex);
                //build path
                pathToDelivery = field.pathToPOI(robot, poi, preferredRoute);
                intakeIndex++;
                break;
            case 1:
                if(pathToDelivery.run(packet)) {
                    //deploy outtake and drive backward very slowly
                    robot.articulate(Robot.Articulation.BACKDROP_PREP);
                    Robot.sensors.distanceSensorsEnabled = true;
                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.1, 0), 0));
                    intakeIndex ++;
                }
                break;
            case 2:
                //todo - change to a more robust exit condition, this assumes pixels have been scored when dist is small
                if(robot.sensors.averageDistSensorValue < 10) {
                    Robot.sensors.distanceSensorsEnabled = false;
                    robot.articulate(Robot.Articulation.TRAVEL_FROM_BACKDROP);
                    outtakeIndex ++;
                }
                break;
            case 3:
                //our job's done here
                if(robot.articulation.equals(Robot.Articulation.TRAVEL)) {
                    outtakeIndex = 0;
                    return true;
                }
                break;
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
        clearStates();
    }
}
