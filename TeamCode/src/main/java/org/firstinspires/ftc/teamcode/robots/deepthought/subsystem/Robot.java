package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.dc;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.field;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.deepthought.DriverControls.fieldOrientedDrive;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.old.Sensors;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DTPosition;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.PositionCache;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.Target;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider.AprilTagProvider;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;


@Config(value = "0_ITD_Robot")
public class Robot implements Subsystem {

    //components and subsystems
    public Subsystem[] subsystems;
    public static Sensors sensors;
    public DriveTrain driveTrain;
    public VisionProvider visionProviderBack, visionProviderFront;
    public static boolean visionOn = true;
    public Trident trident;
    public static boolean updatePositionCache = false;
    public static boolean frontVision = false;

    public boolean calibrating = false;

    public PositionCache positionCache;
    public DTPosition currPosition;

    public DTPosition fetchedPosition;


    //vision variables
    public static boolean visionProviderFinalized = false;
    public static int backVisionProviderIndex = 0;
    public static int frontVisionProviderIndex = 0;
    public double aprilTagRelocalizationX = 0;
    public double aprilTagRelocalizationY = 0;
    //REMOVE
    public Pose2d aprilTagPose = new Pose2d(0, 0, 0);

    private long[] subsystemUpdateTimes;
    private List<LynxModule> hubs = null;
    public HardwareMap hardwareMap;
    private VoltageSensor batteryVoltageSensor;
    public Articulation articulation;
    public List<Target> targets = new ArrayList<Target>();
    public boolean fetched;



    public enum Articulation {
        MANUAL,
        CALIBRATE,
        INTAKE,
        TRAVEL, OUTTAKE
    }

    public void start() {
        if(gameState.equals(IntoTheDeep_6832.GameState.TELE_OP)) {
            trident.shoulder.setPosition(250);
        }
        field.finalizeField(alliance);
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
            driveTrain = new DriveTrain(hardwareMap, this, false);
            trident = new Trident(hardwareMap, this);
            sensors = new Sensors(this);


            subsystems = new Subsystem[]{driveTrain, trident, sensors};
            subsystemUpdateTimes = new long[subsystems.length];

            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
            trident.slideTargetPosition = 0;

            articulation = Articulation.MANUAL;
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
            currPosition = new DTPosition(driveTrain.pose, -trident.shoulder.getCurrentPosition(), trident.slide.getCurrentPosition());
            positionCache.update(currPosition, false);
        }

        articulate(articulation);
        driveTrain.updatePoseEstimate();

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
//
            }
//            if (frontVision) {
//                visionProviderFront.update(debugTelemetryEnabled);
//            } else visionProviderBack.update(debugTelemetryEnabled);
        }
    }


    public void aprilTagRelocalization(int target) {
        ArrayList<AprilTagDetection> detections = getAprilTagDetections();
        if (detections != null && detections.size() > 0) {
            AprilTagDetection targetTag = detections.get(0);
            for (AprilTagDetection detection : detections) {
                if (Math.abs(detection.id - target) < Math.abs(targetTag.id - target))
                    targetTag = detection;
            }

//            aprilTagRelocalizationX = field.getAprilTagPose(targetTag.id).position.x - targetTag.pose.z * 39.37 - DISTANCE_FROM_CAMERA_TO_CENTER_X;
//            aprilTagRelocalizationY = field.getAprilTagPose(targetTag.id).position.y + targetTag.pose.x * 39.37 - DISTANCE_FROM_CAMERA_TO_CENTER_Y;
            aprilTagPose = new Pose2d(targetTag.pose.z, targetTag.pose.x, driveTrain.pose.heading.log());
            driveTrain.setPose(new Pose2d(new Vector2d(aprilTagRelocalizationX, aprilTagRelocalizationY), driveTrain.pose.heading));
            dc.rumble(1, 3000);
            dc.rumble(2, 3000);
        }
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


    public void preloadAllianceSelect() {
        trident.updateColorSensor();
        if(trident.currentSample == Trident.Sample.RED) {
            alliance = Constants.Alliance.RED;
            startingPosition = startingPosition.isRed() == true ?
                    startingPosition :
                    startingPosition == Constants.Position.START_LEFT_BLUE ?
                            Constants.Position.START_LEFT_RED : Constants.Position.START_RIGHT_RED;
        }
        else if(trident.currentSample == Trident.Sample.BLUE) {
            alliance = Constants.Alliance.BLUE;
            startingPosition = startingPosition.isRed() == false ?
                    startingPosition :
                    startingPosition == Constants.Position.START_LEFT_RED ?
                            Constants.Position.START_LEFT_BLUE : Constants.Position.START_RIGHT_BLUE;
        }
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

    public void fetchCachedDTPosition() {
        fetchedPosition = positionCache.readPose();
        fetched = fetchedPosition != null;
    }

    public void resetRobotPosFromCache(double loggerTimeoutMinutes, boolean ignoreCache) {
        if (!ignoreCache) {
            fetchCachedDTPosition();
            if (gameState.equals(IntoTheDeep_6832.GameState.TELE_OP) || gameState.equals((IntoTheDeep_6832.GameState.TEST))) {
                int loggerTimeout = (int) (loggerTimeoutMinutes * 60000);
                if (!(System.currentTimeMillis() - fetchedPosition.getTimestamp() > loggerTimeout || ignoreCache)) {
                    //apply cached position
                    driveTrain.setPose(fetchedPosition.getPose());
                    trident.shoulder.setPosition(fetchedPosition.getShoulderPosition());
                    trident.slide.setPosition(fetchedPosition.getSlidePosition());
                    trident.shoulder.setDirection(DcMotor.Direction.REVERSE);
                }
            }
        }
    }

    public static int calibrateIndex = 0;
    public long calibrateTimer = 0;
    public boolean calibrate() {
        calibrating = true;
        switch (calibrateIndex) {
            case 1:
                calibrateTimer = futureTime(1);
                trident.shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                trident.shoulder.setPower(-.5);
                calibrateIndex++;

            case 2:
                if(Trident.SHOULDER_CALIBRATE_ENCODER == trident.shoulder.getCurrentPosition() && isPast(calibrateTimer)) {
                    calibrateIndex++;
                }
                else {
                    Trident.SHOULDER_CALIBRATE_ENCODER = trident.shoulder.getCurrentPosition();
                }
                break;
            case 3:
                trident.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                trident.shoulder.setTargetPosition(2450);
                trident.shoulder.setPower(1);
                trident.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                calibrateIndex++;

            case 4:
                if(withinError(trident.shoulder.getCurrentPosition(), 2450, 3)) {
                    trident.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    trident.shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
                    trident.shoulder.setPower(1);
                    trident.shoulder.setVelocity(400);
                    trident.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    calibrateIndex++;
                }
                break;
            case 5:
                trident.shoulderTargetPosition = 800;
                calibrateIndex++;
                calibrating = false;
                return true;


        }
        return false;
    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        switch (this.articulation) {
            case MANUAL:
                break;
            case CALIBRATE:
                if (calibrate())
                    articulation = Articulation.MANUAL;
                break;
            case INTAKE:
                trident.sample(alliance);
                if(trident.articulation == Trident.Articulation.MANUAL) {
                    articulation = Articulation.MANUAL;
                }
                break;
            case TRAVEL:
                trident.articulate(Trident.Articulation.TUCK);
                if(trident.articulation == Trident.Articulation.MANUAL) {
                    articulation = Articulation.MANUAL;
                }
                break;
            case OUTTAKE:
                if(outtake())
                    articulation = Articulation.MANUAL;
                break;
        }
        return articulation;
    }

    public int outtakeIndex = 0;
    public boolean outtake() {
        switch (outtakeIndex) {
            case 0:
                Trident.enforceSlideLimits = false;
                trident.articulate(Trident.Articulation.OUTTAKE);
                outtakeIndex++;
                break;
            case 1:
                if(trident.articulation == Trident.Articulation.MANUAL) {
                    Trident.enforceSlideLimits = true;
                    outtakeIndex = 0;
                    return true;
                }
                break;
        }

        return false;
    }


    @Override
    public void stop() {
        currPosition = new DTPosition(driveTrain.pose, -trident.shoulder.getCurrentPosition(), trident.slide.getCurrentPosition());
        positionCache.update(currPosition, true);
        for (Subsystem component : subsystems) {
            component.stop();
        }
    }
    //end stop

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);
        telemetryMap.put("fieldOrientedDrive?", fieldOrientedDrive);
        telemetryMap.put("initPositionIndex", calibrateIndex);
        telemetryMap.put("calibrating", calibrating);
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
