package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.dc;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.field;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.deepthought.DriverControls.fieldOrientedDrive;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.Autonomous;
import org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832;
import org.firstinspires.ftc.teamcode.robots.csbot.util.CSPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.util.PositionCache;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Target;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.AprilTagProvider;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;


@Config(value = "CSRobot")
public class Robot implements Subsystem {

    //components and subsystems
    public Subsystem[] subsystems;
    public static Sensors sensors;
    public DriveTrain driveTrain;
    public static boolean drawFieldEnabled = true;

    public Skyhook skyhook;
    public Intake intake;
    public VisionProvider visionProviderBack, visionProviderFront;
    public static boolean visionOn = true;
    public Outtake outtake;
    public static boolean updatePositionCache = false;
    public static boolean frontVision = false;
    public PositionCache positionCache;
    public CSPosition currPosition;

    public CSPosition fetchedPosition;
    //relocalization
    public boolean forward = false;

    //vision variables
    public static boolean visionProviderFinalized = false;
    public static int backVisionProviderIndex = 0;
    public static int frontVisionProviderIndex = 0;
    public double aprilTagRelocalizationX = 0;
    public double aprilTagRelocalizationY = 0;
    //REMOVE
    public Pose2d aprilTagPose = new Pose2d(0, 0, 0);

    public static double DISTANCE_FROM_CAMERA_TO_CENTER_X = 16;// In inches
    public static double DISTANCE_FROM_CAMERA_TO_CENTER_Y = 5.25;
    public boolean initing = false;

    private long[] subsystemUpdateTimes;
    private List<LynxModule> hubs = null;
    public HardwareMap hardwareMap;
    private VoltageSensor batteryVoltageSensor;
    public Articulation articulation;
    public List<Target> targets = new ArrayList<Target>();
    public boolean fetched;

    public void backdropRelocalize() {
        driveTrain.RELOCALIZE_WITH_IMU = false;
        distanceSensorHeading();
        frontVision = false;
        if (backVisionProviderIndex == 0) {
            //assumes that the backvision is in apriltag
            enableVision();
            if (((AprilTagProvider) visionProviderBack).getDetections() != null) {
                aprilTagRelocalization(Autonomous.targetAprilTagIndex);
            }
        } else {
            visionProviderFinalized = false;
            backVisionProviderIndex = 0;
        }
        //say bye bye to loop times
    }

    public void distanceSensorX() {
//        Vector2d newLocation = new Vector2d(field.APRILTAG1.pose.position.x - (driveTrain.leftDistanceSensorValue + 7), driveTrain.pose.position.y);
//        driveTrain.pose = new Pose2d(newLocation, driveTrain.pose.heading);
    }

    public void distanceSensorHeading() {
        Sensors.distanceSensorsEnabled = true;
        if (sensors.averageDistSensorValue < 30) {
            double distDiff = Math.abs(sensors.rightDistSensorValue - sensors.leftDistSensorValue);
            double heading = Math.PI -
                    Math.asin(distDiff / Math.hypot(driveTrain.DISTANCE_BETWEEN_DISTANCE_SENSORS, distDiff))
                            * ((sensors.leftDistSensorValue > sensors.rightDistSensorValue) ? 1 : 1);
            driveTrain.setPose(new Pose2d(driveTrain.pose.position, heading));
            Sensors.distanceSensorsEnabled = false;
        }
    }

    public enum Articulation {
        //beater bar, drivetrain, drone launcher, outtake
        MANUAL,
        CALIBRATE,
        BACKDROP_PREP,
        BACKDROP,
        INGEST,
        HANG,
        PREP_FOR_HANG,
        LAUNCH_DRONE,
        TRAVEL_FROM_BACKDROP,
        TRAVEL_FROM_INGEST,
        TRAVEL,
        LINE;
    }

    public void start() {
        if (!gameState.isAutonomous())
            skyhook.articulate(Skyhook.Articulation.TRAVEL);
        //TODO - articulate starting position
    }
    //end start


    public Robot(HardwareMap hardwareMap, boolean simulated) {
        if (!simulated) {
            this.hardwareMap = hardwareMap;
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule module : hubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            //initialize vision
            createVisionProviders();

            positionCache = new PositionCache(5);

            // initializing subsystems
            driveTrain = new DriveTrain(hardwareMap, this, simulated);
            intake = new Intake(hardwareMap, this);
            outtake = new Outtake(hardwareMap, this);
            skyhook = new Skyhook(hardwareMap, this);
            sensors = new Sensors(this);


            subsystems = new Subsystem[]{driveTrain, intake, outtake, skyhook, sensors};
            subsystemUpdateTimes = new long[subsystems.length];

            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            articulation = Articulation.MANUAL;
            backVisionProviderIndex = 0;
        }

    }
    //end constructor

    public double deltaTime = 0;
    long lastTime = 0;

    @Override
    public void update(Canvas fieldOverlay) {
        deltaTime = (System.nanoTime() - lastTime) / 1e9;
        lastTime = System.nanoTime();
        clearBulkCaches(); //ALWAYS FIRST LINE IN UPDATE

        if (updatePositionCache && gameState.isAutonomous()) {
            currPosition = new CSPosition(driveTrain.pose, skyhook.getSkyhookLeftTicksCurrent(), skyhook.getSkyhookRightTicksCurrent());
            positionCache.update(currPosition, false);
        }

        articulate(articulation);
        driveTrain.updatePoseEstimate();

        if (drawFieldEnabled)
            drawRobot(fieldOverlay, driveTrain.pose);

        //update subsystems
        for (int i = 0; i < subsystems.length; i++) {
            Subsystem subsystem = subsystems[i];
            long updateStartTime = System.nanoTime();
            subsystem.update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }
    }
    //end update

    public void enableVision() {
        if (visionOn) {
            if (!visionProviderFinalized) {
                createVisionProviders();
                visionProviderBack.initializeVision(hardwareMap, this, false);
                visionProviderFront.initializeVision(hardwareMap, this, true);
                visionProviderFinalized = true;

            }
            if (frontVision) {
                visionProviderFront.update(debugTelemetryEnabled);
            } else visionProviderBack.update(debugTelemetryEnabled);
        }
    }


    public void aprilTagRelocalization(int target) {
//        ArrayList<AprilTagDetection> detections = getAprilTagDetections();
//        if (detections != null && detections.size() > 0) {
//            AprilTagDetection targetTag = detections.get(0);
//            for (AprilTagDetection detection : detections) {
//                if (Math.abs(detection.id - target) < Math.abs(targetTag.id - target))
//                    targetTag = detection;
//            }

//            aprilTagRelocalizationX = field.getAprilTagPose(targetTag.id).position.x - targetTag.pose.z * 39.37 - DISTANCE_FROM_CAMERA_TO_CENTER_X;
//            aprilTagRelocalizationY = field.getAprilTagPose(targetTag.id).position.y + targetTag.pose.x * 39.37 - DISTANCE_FROM_CAMERA_TO_CENTER_Y;
//            aprilTagPose = new Pose2d(targetTag.pose.z, targetTag.pose.x, driveTrain.pose.heading.log());
//            driveTrain.pose = new Pose2d(new Vector2d(aprilTagRelocalizationX, aprilTagRelocalizationY), driveTrain.pose.heading);
//            dc.rumble(1, 3000);
//            dc.rumble(2, 3000);
        }


    public ArrayList<AprilTagDetection> getAprilTagDetections() {
        if (visionOn) {
            if (visionProviderFinalized) {
                return ((AprilTagProvider) visionProviderBack).getDetections();
            }
        }
        return null;
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 8;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public void switchVisionProviders() {
        visionProviderBack.shutdownVision();
        visionProviderFront.shutdownVision();
        if (backVisionProviderIndex == 2) {
            //switch to AprilTags
            backVisionProviderIndex = 0;
            visionProviderFinalized = false;

        } else if (backVisionProviderIndex == 0) {
            //switch back to ColorBlob
            backVisionProviderIndex = 2;
            visionProviderFinalized = false;
        }
    }

    public void fetchCachedCSPosition() {
        fetchedPosition = positionCache.readPose();
        fetched = fetchedPosition != null;
    }

    public void resetRobotPosFromCache(double loggerTimeoutMinutes, boolean ignoreCache) {
        if (!ignoreCache) {
            fetchCachedCSPosition();
            if (gameState.equals(CenterStage_6832.GameState.TELE_OP) || gameState.equals((CenterStage_6832.GameState.TEST))) {
                int loggerTimeout = (int) (loggerTimeoutMinutes * 60000);
                if (!(System.currentTimeMillis() - fetchedPosition.getTimestamp() > loggerTimeout || ignoreCache)) {
                    //apply cached position
//                    driveTrain.pose = fetchedPosition.getPose();
                }
            }
        }
    }

    public static int initPositionIndex = 0;
    public long initPositionTimer;

    public void initPosition() {
        switch (initPositionIndex) {
            case 0:
                break;
            case 1:
                initing = true;
                intake.articulate(Intake.Articulation.MANUAL);
                initPositionTimer = futureTime(1);
                skyhook.skyhookLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                skyhook.skyhookRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                skyhook.skyhookRight.setPower(-1);
                skyhook.skyhookLeft.setPower(-1);
                initPositionIndex++;
                break;
            case 2:
                intake.setAngle(Intake.ANGLE_GROUND);
//                if(isPast(initPositionTimer)){
                initPositionIndex++;
//                }
                break;
            case 3:
                outtake.slideTargetPosition = Outtake.UNTUCK_SLIDE_POSITION;
                outtake.setTargetAngle(Outtake.ELBOW_START_ANGLE, Outtake.WRIST_INIT_ANGLE, Outtake.ELEVATOR_START_ANGLE);
                Sensors.touchSensorsEnabled = true;
//                skyhook.skyhookLeft.setVelocity(100);
//                skyhook.skyhookRight.setVelocity(100);
                initPositionIndex++;
//                if (isPast(initPositionTimer)) {
//                    initPositionTimer = futureTime(1);
//                    initPositionIndex ++;
//                }
                break;
            case 4:
                if (sensors.rightTouchSensor) {
                    skyhook.skyhookRight.setPower(0);

                } else skyhook.skyhookRight.setPower(-.08);

                if (sensors.leftTouchSensor) {
                    skyhook.skyhookLeft.setPower(0);
                } else skyhook.skyhookLeft.setPower(-.2);

                if (sensors.leftTouchSensor && sensors.rightTouchSensor) {
                    skyhook.skyhookRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    skyhook.skyhookRight.setVelocity(0);
                    skyhook.skyhookRight.setPower(Skyhook.SKYHOOK_POWER);
                    skyhook.skyhookRight.setTargetPosition(0);
                    skyhook.skyhookRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    skyhook.skyhookRight.setPosition(0);
                    skyhook.skyhookLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    skyhook.skyhookLeft.setVelocity(0);
                    skyhook.skyhookLeft.setPower(Skyhook.SKYHOOK_POWER);
                    skyhook.skyhookLeft.setTargetPosition(0);
                    skyhook.skyhookLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    skyhook.skyhookLeft.setPosition(0);
                    Sensors.touchSensorsEnabled = false;
                    initPositionIndex++;
                }
                //                if (isPast(initPositionTimer)) {
//                    initPositionTimer = futureTime(1);
//                    initPositionIndex ++;
//                }
                break;
            case 5:
                outtake.slideTargetPosition = 0;
                initPositionIndex++;
                break;
            case 6:
                //todo load cached skyhook positions
                //this is the only way to work across power cycles until we incorporate a limit switch calibration
//                skyhook.skyhookLeft.setPosition(0);
//                skyhook.skyhookRight.setPosition(0);
                initing = false;
                skyhook.articulate(Skyhook.Articulation.INIT);
                break;
            case 7:
                if (gameState.equals(CenterStage_6832.GameState.RELOCALIZATION_TEST)) {
                    backVisionProviderIndex = 0;
                    visionProviderFinalized = false;
                }
                intake.articulate(Intake.Articulation.INIT);
            default:
                break;
        }
    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        switch (this.articulation) {
            case LINE:
                lineTest();
                break;
            case MANUAL:
                break;
            case TRAVEL:
//                robot.outtake.articulate(Outtake.Articulation.TRAVEL);
                cleanArticulations();
                break;
            case BACKDROP:
                break;
            case CALIBRATE:
                //TODO - WRITE A CALIBRATION ROUTINE
                break;
            case INGEST:
                if (ingest()) {
                    articulation = Articulation.TRAVEL_FROM_INGEST;
                }
                break;
            case HANG:
                intake.articulate(Intake.Articulation.HANG);
                skyhook.articulate(Skyhook.Articulation.HANG);
                break;
            case PREP_FOR_HANG:
                skyhook.articulate(Skyhook.Articulation.PREP_FOR_HANG);
                outtake.articulate(Outtake.Articulation.INGEST_FROM_TRAVEL);
                articulation = Articulation.TRAVEL;
                break;
            case LAUNCH_DRONE:
//                if(driveTrain.driveToDrone()) {
                skyhook.articulate(Skyhook.Articulation.LAUNCH);
                articulation = Articulation.MANUAL;
//                }
                break;
            case TRAVEL_FROM_INGEST:
                intake.articulate(Intake.Articulation.TRAVEL);
                articulation = Articulation.TRAVEL;
//                if (!(outtake.articulation==Outtake.Articulation.TRAVEL)) {
//                    outtake.articulate(Outtake.Articulation.TRAVEL_FROM_INGEST);
//                }
                break;
            case TRAVEL_FROM_BACKDROP:
                if (travelFromBackdrop()) {
                    articulation = Articulation.TRAVEL;
                }
                break;
            case BACKDROP_PREP:
                if (backdropPrep()) {
                    intake.articulate(Intake.Articulation.TRAVEL);
                    articulation = Articulation.BACKDROP;
                }
                break;
        }
        return articulation;
    }

    int lineTestIndex = 0;
    long lineTestTimer = 0;

    public void lineTest() {
        switch (lineTestIndex) {
            case 0:
                forward = false;
                lineTestTimer = futureTime(2);
                driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.4, 0), 0));
                lineTestIndex++;
                break;
            case 1:
                if (isPast(lineTestTimer)) {
                    forward = true;
                    driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(.4, 0), 0));
                    lineTestTimer = futureTime(2);
                    lineTestIndex++;
                }
                break;
            case 2:
                if (isPast(lineTestTimer))
                    lineTestIndex = 0;
                forward = false;
                break;

        }
    }

    int backdropPrepIndex = 0;
    long backdropPrepTimer = 0;

    private boolean backdropPrep() {
        switch (backdropPrepIndex) {
            case 0:
                intake.articulate(Intake.Articulation.EJECT);
                outtake.articulate(Outtake.Articulation.BACKDROP_PREP);
                backdropPrepTimer = futureTime(.5);
                backdropPrepIndex++;
                break;
            case 1:
                if (isPast(backdropPrepTimer)) {
                    backdropPrepIndex = 0;
                    return true;
                }
        }
        return false;
    }

    public void toggleBackdropPrep() {
        if (articulation.equals(Articulation.BACKDROP)) {
            articulation = Articulation.TRAVEL_FROM_BACKDROP;
        } else {
            articulation = Articulation.BACKDROP_PREP;
        }
    }

    @Override
    public void stop() {
        currPosition = new CSPosition(driveTrain.pose, skyhook.getSkyhookLeftTicksCurrent(), skyhook.getSkyhookRightTicksCurrent());
        positionCache.update(currPosition, true);
        for (Subsystem component : subsystems) {
            component.stop();
        }
    }
    //end stop

    public void cleanArticulations() {
        if (articulation == Articulation.TRAVEL) {
            ingestStage = 0;
            if (intake.articulation == Intake.Articulation.TRAVEL)
                intake.cleanArticulations();
            if (outtake.articulation == Outtake.Articulation.TRAVEL)
                outtake.cleanArticulations();
        }
    }

    public static int travelFromBackdropStage = 0;
    public static long travelFromBackdropTimer = 0;

    public boolean travelFromBackdrop() {
        switch (travelFromBackdropStage) {
            case 0:
                intake.articulate(Intake.Articulation.DOWN);
                travelFromBackdropStage++;
                break;
            case 1:
                outtake.articulate(Outtake.Articulation.TRAVEL_FROM_BACKDROP);
                travelFromBackdropTimer = futureTime(2);
                travelFromBackdropStage++;
                break;
            case 2:
                if (isPast(travelFromBackdropTimer)) {
                    intake.articulate(Intake.Articulation.TRAVEL);
                    travelFromBackdropStage = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    public static int ingestStage = 0;
    public long ingestTimer = 0;

    public boolean ingest() {
        switch (ingestStage) {
            case 0:
                outtake.articulate(Outtake.Articulation.INGEST_FROM_TRAVEL);
                ingestTimer = futureTime(.5);
                ingestStage++;
            case 1: //wait for outake to dock before proceeding
                if (isPast(ingestTimer)) {
                    //intake can start eating
                    intake.articulate(Intake.Articulation.INGEST);
                    ingestStage++;
                }
                break;
            case 2:  //wait until Intake ingest and swallow are done
                if (!intake.isEating()) ingestStage++;
                break;
            case 3:
                ingestStage = 0;
                return true;

        }
        return false;
    }

    public void enterTravel() {
        if (articulation.equals(Articulation.BACKDROP)) {
            articulation = Articulation.TRAVEL_FROM_BACKDROP;
        }
        if (articulation.equals(Articulation.INGEST)) {
            articulation = Articulation.TRAVEL_FROM_INGEST;
        } else {
            articulation = Articulation.TRAVEL;
        }
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);
        telemetryMap.put("fieldOrientedDrive?", fieldOrientedDrive);
        telemetryMap.put("wingIntakeIndex", ingestStage);
        telemetryMap.put("initPositionIndex", initPositionIndex);
        telemetryMap.put("Vision On/Vision Provider Finalized", visionOn + " " + visionProviderFinalized);
        telemetryMap.put("april tag relocalization point", "(" + aprilTagRelocalizationX + ", " + aprilTagRelocalizationY + ")");
        telemetryMap.put("april tag pose", "(" + aprilTagPose.position.x * 39.37 + ", " + aprilTagPose.position.y * 39.37 + ")");
//        telemetryMap.put("MemoryPose", positionCache.readPose());
        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
        }


        telemetryMap.put("Delta Time", deltaTime);


        return telemetryMap;
    }
    //end getTelemetry

    public void createVisionProviders() {
        try {
            visionProviderBack = VisionProviders.VISION_PROVIDERS[backVisionProviderIndex].newInstance().setRedAlliance(alliance.isRed());
            visionProviderFront = VisionProviders.VISION_PROVIDERS[frontVisionProviderIndex].newInstance().setRedAlliance(alliance.isRed());
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }

    public void clearBulkCaches() {
        for (LynxModule module : hubs)
            module.clearBulkCache();
    }

    public double getVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    @Override
    public String getTelemetryName() {
        return "ROBOT";
    }
}
