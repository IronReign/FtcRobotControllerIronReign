package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.lebot2.Robot;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Loader;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Vision;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain.DriveTrainBase;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain.TankDrive;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "Lebot2_Autonomous")
public class Autonomous implements TelemetryProvider{

    // Configuration
    private final Robot robot;

    // Constants
    public int rows = 0;

    // Autonomous sequence states
    public enum AutonSequence {
        IDLE,               // Resets encoders
        STARTING_POSITION,  // Backwards movement to shooting position
        TARGETING_GOAL,     // Adjusts to face tag
        LAUNCH_ALL,         // Shoots all balls
        INTAKE_POSITION,    // Moves backwards towards balls
        TARGETING_BALLS,    // Turns towards balls on floor
        INTAKE,             // Intakes 3 balls
        END                 // Waits after completed
    }

    private AutonSequence autonSequence = AutonSequence.IDLE;
    private AutonSequence previousAutonSequence = AutonSequence.IDLE;

    @Override
    public String getTelemetryName() {
        return "Autonomous";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Auton State", autonSequence);

        return telemetry;
    }

    public Autonomous (Robot robot){
        this.robot = robot;
    }

    public void execute(){

        // Alliance correction
        if(robot.isRedAlliance){
            robot.headingDegrees *= -1; // will this pose an error?
        }

        switch(autonSequence){
            case IDLE:
                robot.driveTrain.resetEncoders();
                autonSequence(AutonSequence.STARTING_POSITION);
                break;

            case STARTING_POSITION:
                // Move backwards to starting shooting position
                robot.driveTrain.drive(-0.7, 0, 0);

                int leftTicks = Math.abs(robot.driveTrain.getLeftTicks());

                if(leftTicks >= 2195){
                    robot.driveTrain.drive(0, 0, 0);
                    autonSequence(AutonSequence.TARGETING_GOAL);
                }
                break;

            case TARGETING_GOAL:
                // Adjust to face the goal
                robot.handleTargetingArticulation();

                if(robot.getArticulation() == Robot.Articulation.MANUAL){
                    autonSequence(AutonSequence.LAUNCH_ALL);
                }
                break;

            case LAUNCH_ALL:
                // Launch all loaded balls
                robot.handleLaunchAllArticulation();

                if (robot.getArticulation() == Robot.Articulation.MANUAL){
                    robot.driveTrain.resetEncoders();
                    autonSequence(AutonSequence.INTAKE_POSITION);
                }
                break;

            case INTAKE_POSITION:
                // Move backwards to position near three balls

                robot.driveTrain.drive(-0.6, 0, 0);

                int leftTicksR2 = Math.abs(robot.driveTrain.getLeftTicks());

                if(leftTicksR2 >= 800){ //TODO: Measure ticks between two rows
                    robot.driveTrain.drive(0, 0, 0);
                    autonSequence(AutonSequence.TARGETING_BALLS);
                }
                break;

            case TARGETING_BALLS:
                // Turn towards three balls on floor (90 degrees?)
                robot.driveTrain.turnToHeading(90, 0.6); //TODO: Change headingDegrees

                if (robot.driveTrain.isTurnComplete()){
                    autonSequence(AutonSequence.INTAKE);
                }
                break;

            case INTAKE:
                // Intake the three balls
                robot.handleIntakeArticulation();
                robot.driveTrain.drive(0.1, 0, 0);

                if (robot.getArticulation() == Robot.Articulation.MANUAL){
                    robot.driveTrain.drive(0,0,0);
                    rows++;
                    if (rows<=1){ //adjust later if works
                        autonSequence(AutonSequence.TARGETING_GOAL);
                    } else {
                        autonSequence(AutonSequence.END);
                    }
                }
                break;

            case END:
                // Wait until time is over
                break;
        }
    }

    public void autonSequence(AutonSequence newAutonSequence) {
        if (autonSequence != newAutonSequence) {
            previousAutonSequence = autonSequence;
            autonSequence = newAutonSequence;
        }
    }
}
