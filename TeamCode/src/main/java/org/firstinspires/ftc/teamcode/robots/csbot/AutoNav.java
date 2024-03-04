package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;

public class AutoNav {
    Robot robot;
    Field field;
    public static boolean cycling;
    public static boolean intaking;
    //todo - allow manipulation of this
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
                if (intake(packet, preferredRoute, true))
                    intaking = false;
            }
            else {
                //assuming middle apriltag for now
                if(outtake(packet, preferredRoute, 2));
                    intaking = true;
            }
        }
        dashboard.sendTelemetryPacket(packet);
    }

    public void clearStates() {
        intakeIndex = 0;
        outtakeIndex = 0;
    }

    SequentialAction pathToIntake;
    int intakeIndex = 0;
    public boolean intake(TelemetryPacket packet, int preferredRoute, boolean wing) {
        switch (intakeIndex) {
            case 0:
                //get poi for desired intake location, assuming wing for now
//                if(wing)
                POI poi = field.WING_INTAKE;
                //build path
                pathToIntake = field.pathToPOI(robot, poi, preferredRoute);
                intakeIndex++;
                break;
            case 1:
                if(pathToIntake.run(packet)) {
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

    public SequentialAction pathToOuttake;
    int outtakeIndex = 0;
    public boolean outtake(TelemetryPacket packet, int preferredRoute, int targetIndex) {
        switch (outtakeIndex) {
            case 0:
                //select outtake score locations
                POI poi = alliance.getMod()? field.scoreLocations.get(targetIndex + 3) : field.scoreLocations.get(targetIndex);
                //build path
                pathToOuttake = field.pathToPOI(robot, poi, preferredRoute);
                intakeIndex++;
                break;
            case 1:
                if(pathToOuttake.run(packet)) {
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

}
