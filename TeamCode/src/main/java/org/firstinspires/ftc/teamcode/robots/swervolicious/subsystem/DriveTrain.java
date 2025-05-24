package org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem;


import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.old.Sensors.distanceSensorsEnabled;
import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.robots.swervolicious.IntoTheSwerve_6832;
import org.firstinspires.ftc.teamcode.robots.swervolicious.rr_localize.MecanumDriveReign;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
import org.firstinspires.ftc.teamcode.robots.swervolicious.rr_localize.SwerveDriveReign;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.HashMap;
import java.util.Map;

@Config(value = "0_Swerve_Drive_Train")
public class DriveTrain extends SwerveDriveReign implements Subsystem {
    public Robot robot;
    public boolean trajectoryIsActive;
    public static double GLOBAL_HEADING_DAMPENING = .7;
    public static double HEADING_DAMPENING = 0.4;
    public static double DRIVE_DAMPENING = 0.4;
    public DistanceSensor leftDistanceSensor, rightDistanceSensor;
    public double rightDistanceSensorValue = 0;
    public double leftDistanceSensorValue = 0;
    public double imuRoadrunnerError;
    public static boolean roadRunnerDrive = true;

    public boolean imuTurnDone = false;
    public static boolean runTestPath = false;
    private double targetHeading, targetVelocity = 0;
    public static PIDController headingPID;
    public static PIDCoefficients HEADING_PID_PWR = new PIDCoefficients(0.03, 0.04, 0);
    public static double HEADING_PID_TOLERANCE = 3.5; //this is a percentage of the input range .063 of 2PI is 1 degree
    private double PIDCorrection, PIDError;
    public static int turnToTest = 0;
    public static double turnToSpeed = .8; //max angular speed for turn
    public static double DISTANCE_BETWEEN_DISTANCE_SENSORS = 14;
    public boolean RELOCALIZE_WITH_IMU = false;

    public static SequentialAction testPathToWing, testPathToScore;


    public DriveTrain(HardwareMap hardwareMap, Robot robot, boolean simulated) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        this.robot = robot;
        trajectoryIsActive = false;
//        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistRight");
//        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistLeft");
        headingPID = new PIDController(HEADING_PID_PWR);
        headingPID.setInputRange(0, 360);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();

    }
    //end constructor

    @Override
    public void update(Canvas c) {

        if (roadRunnerDrive) {
//            imuRoadrunnerError = Robot.sensors.driveIMUYaw - Math.toDegrees(pose.heading.log());
//            if (distanceSensorsEnabled) {
//                rightDistanceSensorValue = Robot.sensors.rightDistSensorValue;
//                leftDistanceSensorValue = Robot.sensors.leftDistSensorValue;
//            }
            updatePoseEstimate();
            super.updateModules();

//        update pose heading from imu regularly
//            if (RELOCALIZE_WITH_IMU) {
//                if ((int) (System.nanoTime() / 1e9) % 2 == 0) {
//                    setPose(new Pose2d(pose.position, Math.toRadians(Robot.sensors.driveIMUYaw)));
//                }
//            }

            //test imu based turning from dashboard - todo comment out when not needed
//            if (turnToTest != 0) turnUntilDegreesIMU(turnToTest, turnToSpeed); //can target any angle but zero
        }

    }


    public void drive(double x, double y, double theta) {

        trajectoryIsActive = false;
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -y, //todo check if these should be reversed
                        -x
                ),
                theta
        ));
        updatePoseEstimate();
    }

    public void fieldOrientedDrive(double x, double y, double theta, boolean isRed) {
        theta = theta * GLOBAL_HEADING_DAMPENING;

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                x,
                y);
        //additional rotation needed if blue alliance perspective
        Rotation2d heading = (!isRed) ? localizer.getPose().heading : localizer.getPose().heading.plus(Math.PI);
        input = heading.inverse().times(
                new Vector2d(-input.x, input.y));
        setDrivePowers(new PoseVelocity2d(input, -theta));
        updatePoseEstimate();
    }

    //request a turn in degrees units
    //this is an absolute (non-relative) implementation.
    //the direction of the turn will favor the shortest approach
    public boolean turnUntilDegreesIMU(double turnAngle, double maxSpeed) {
        targetHeading = wrapAngle(turnAngle);
        headingPID.setPID(HEADING_PID_PWR);
        //headingPID.setInput(wrapAngle(Robot.sensors.driveIMUYaw));
        headingPID.setInput(wrapAngle(Utils.wrapAngle(Math.toDegrees(localizer.getPose().heading.log())))); //get heading in degrees from current localizer
        headingPID.setSetpoint(targetHeading);
        headingPID.setOutputRange(-maxSpeed, maxSpeed);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        double correction = headingPID.performPID();
        PIDCorrection = correction;
        PIDError = headingPID.getError();
        if (headingPID.onTarget()) {
            //turn meets accuracy target
            //todo is this a good time to update getPose() heading from imu?
            //setPose(new Pose2d(getPose().position, Math.toRadians(Robot.sensors.driveIMUYaw)));
            //stop
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return imuTurnDone = true;
        } else {
            headingPID.enable();
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), correction));
            return imuTurnDone = false;
        }
    }

    public void setPose(Constants.Position start) {
        setPose(start.getPose());
    }

    public void setPose(Pose2d pose) {
        super.localizer.setPose(pose);
    }

    /**
     * Exposes the current and target angles of each swerve module for telemetry.
     * @return array of ModuleState objects (current + target angle in radians).
     */
    public SwerveDriveReign.ModuleState[] getModuleStates() {
        return super.getModuleStates();
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public void resetStates() {

    }

    @Override
    public boolean calibrate() { //stub until we need an in-game drivetrain calibration
        return true;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {

        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("x in fieldCoords", localizer.getPose().position.x / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("y in fieldCoords", localizer.getPose().position.y / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("x in inches", localizer.getPose().position.x);
        telemetryMap.put("y in inches", localizer.getPose().position.y);

//        telemetryMap.put("Right Distance Sensor Value", robot.sensors.rightDistSensorValue);
//        telemetryMap.put("Left Distance Sensor Value", robot.sensors.leftDistSensorValue);

        telemetryMap.put("heading", Math.toDegrees(localizer.getPose().heading.log()));

// these were for the mecanum drive
//        telemetryMap.put("Left Odometry Pod:\t", leftFront.getCurrentPosition());
//        telemetryMap.put("Right Odometry Pod:\t", rightFront.getCurrentPosition());
//        telemetryMap.put("Cross Odometry Pod:\t", rightBack.getCurrentPosition());
        telemetryMap.put("imu vs roadrunner:\t", imuRoadrunnerError);
//        telemetryMap.put("imu:\t", Robot.sensors.driveIMUYaw);
        telemetryMap.put("PID Error:\t", PIDError);
        telemetryMap.put("PID Correction:\t", PIDCorrection);
        telemetryMap.put("imuTurnDone?", imuTurnDone);
        // these were for the mecanum drive
//        telemetryMap.put("Left Front Motor Power", leftFront.getPower());
//        telemetryMap.put("Left Back Motor Power", leftBack.getPower());
//        telemetryMap.put("Right Front Motor Power", rightFront.getPower());
//        telemetryMap.put("Right Back Motor Power", rightBack.getPower());
        telemetryMap.put("distanceSensorsEnabled?", distanceSensorsEnabled);
        telemetryMap.put("testPath?", testPathToWing);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "DRIVETRAIN";
    }

    int strafeAndTurnIndex = 0;
    SequentialAction strafeAndTurnAction;
    public boolean strafeAndTurn(Pose2d pose2d, TelemetryPacket packet) {
        switch (strafeAndTurnIndex) {
            case 0:
                TrajectoryActionBuilder strafeAndTurnActionBuilder = robot.driveTrain.actionBuilder(localizer.getPose())
                        .strafeToLinearHeading(pose2d.position, pose2d.heading);
                strafeAndTurnAction = new SequentialAction(strafeAndTurnActionBuilder.build());
                strafeAndTurnIndex++;
                break;
            case 1:
                robot.driveTrain.trajectoryIsActive = true;
                if (!strafeAndTurnAction.run(packet)) {
                    strafeAndTurnIndex = 0;
                    robot.driveTrain.trajectoryIsActive = false;
                    return true;
                }
                break;
        }
        return false;
    }

    int strafeToPoseIndex = 0;
    SequentialAction strafeToPoseAction;

    public boolean strafeToPose(Pose2d pose2d, TelemetryPacket packet) {
        if (roadRunnerDrive) {
            switch (strafeToPoseIndex) {
                case 0:
                    TrajectoryActionBuilder strafeActionBuilder = robot.driveTrain.actionBuilder(localizer.getPose())
                            .strafeTo(pose2d.position)
                            .turnTo(pose2d.heading);
                    strafeToPoseAction = new SequentialAction(strafeActionBuilder.build());
                    strafeToPoseIndex++;
                    break;
                case 1:
                    robot.driveTrain.trajectoryIsActive = true;
                    if (!strafeToPoseAction.run(packet)) {
                        strafeToPoseIndex = 0;
                        robot.driveTrain.trajectoryIsActive = false;
                        return true;
                    }
                    break;
            }
        }
        return false;
    }


    int turnThenStrafeIndex = 0;
    SequentialAction turnThenStrafeAction;

    public boolean turnThenStrafe(Pose2d pose2d, TelemetryPacket packet) {
        if (roadRunnerDrive) {
            switch (turnThenStrafeIndex) {
                case 0:
                    TrajectoryActionBuilder strafeActionBuilder = robot.driveTrain.actionBuilder(localizer.getPose())
                            .turnTo(pose2d.heading)
                            .strafeTo(pose2d.position);

                    turnThenStrafeAction = new SequentialAction(strafeActionBuilder.build());
                    turnThenStrafeIndex++;
                    break;
                case 1:
                    robot.driveTrain.trajectoryIsActive = true;
                    if (!turnThenStrafeAction.run(packet)) {
                        turnThenStrafeIndex = 0;
                        robot.driveTrain.trajectoryIsActive = false;
                        return true;
                    }
                    break;
            }
        }
        return false;
    }
}
