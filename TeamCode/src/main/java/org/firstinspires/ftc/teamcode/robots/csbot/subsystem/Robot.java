package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.field;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.csbot.DriverControls.fieldOrientedDrive;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832;
import org.firstinspires.ftc.teamcode.robots.csbot.util.CSPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
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


@Config(value = "AA_CSRobot")
public class  Robot implements Subsystem {

    //components and subsystems
    public Subsystem[] subsystems;
    public DriveTrain driveTrain;
    public Skyhook skyhook;
    public Intake intake;
    public VisionProvider visionProviderBack, visionProviderFront;
    public static boolean visionOn = true;
    public Outtake outtake;
    public static boolean updatePositionCache = false;
    public PositionCache positionCache;
    public CSPosition currPosition;

    public CSPosition fetchedPosition;

    //vision variables
    public boolean visionProviderFinalized = false;
    public static int visionProviderIndex = 2;
    public double aprilTagRelocalizationX = 0;
    public double aprilTagRelocalizationY = 0;
    //REMOVE
    public Pose2d aprilTagPose = new Pose2d(0, 0,0);

    public static double DISTANCE_FROM_CAMERA_TO_CENTER_X = 13;// In inches

    private long[] subsystemUpdateTimes;
    private final List<LynxModule> hubs;
    public HardwareMap hardwareMap;
    private VoltageSensor batteryVoltageSensor;
    public Articulation articulation;
    public List<Target> targets = new ArrayList<Target>();
    public boolean fetched;
    //auton end game variables
    private int driveToDroneIndex = 0;
    public boolean autoEndgame = false;

    public boolean selfDriving = true;

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
        TRAVEL

    }

    public void start() {
        if(!gameState.isAutonomous())
            skyhook.articulate(Skyhook.Articulation.GAME);
        //TODO - articulate starting position
    }
    //end start


    public Robot(HardwareMap hardwareMap, boolean simulated) {
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


        subsystems = new Subsystem[]{driveTrain, intake, outtake, skyhook};
        subsystemUpdateTimes = new long[subsystems.length];

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        articulation = Robot.Articulation.MANUAL;
         visionProviderIndex = 2;

//        field = new Field(true);
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

        if(autoEndgame && driveToDrone()) { //no no I swear this works
            autoEndgame = false;
        }

        articulate(articulation);
        //TODO - DELETE
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
                visionProviderBack.initializeVision(hardwareMap, this);
                visionProviderFinalized = true;

            }
            visionProviderBack.update();
        }
    }

    private Action driveToDrone;
    public void driveToDroneBuild() {
        driveToDrone = new SequentialAction(
                driveTrain.actionBuilder(driveTrain.pose)
                        .splineTo(new Vector2d(23.5, CenterStage_6832.startingPosition.getMod()?-33:33), 0)
                        .splineTo(new Vector2d(0, CenterStage_6832.startingPosition.getMod()?-33:33), 0)
                        .build()
        );
    }

    private long futureTimer = 0;
    public boolean driveToDrone() {
        switch (driveToDroneIndex) {
            case 0:
                if (driveToDrone == null) {
                    driveToDroneBuild();
                }
                else if (!driveToDrone.run(new TelemetryPacket())) {
                    driveToDrone = null;
                    driveToDroneIndex++;
                }
                break;
            case 1:
                articulate(Robot.Articulation.LAUNCH_DRONE);
                futureTimer = futureTime(5);
                driveToDroneIndex++;
                break;
            case 2:
                if (isPast(futureTimer)) {
                    articulate(Robot.Articulation.PREP_FOR_HANG);
                    futureTimer = futureTime(2);
                    driveToDroneIndex++;
                }
                break;
            case 3:
                driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.2, 0), 0));
                if (isPast(futureTimer)) {
                    driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    driveToDroneIndex++;
                }
                break;
            case 4:
                if (skyhook.articulation.equals(Skyhook.Articulation.PREP_FOR_HANG)) {
                    articulate(Robot.Articulation.HANG);
                    driveToDroneIndex = 0;
                    return true;
                }
        }
            return false;
    }

    public void aprilTagRelocalization(int target) {
        ArrayList<AprilTagDetection> detections = getAprilTagDetections();
        if(detections != null && detections.size() > 0) {
            AprilTagDetection targetTag = detections.get(0);
            for (AprilTagDetection detection : detections) {
                if(Math.abs(detection.id-target) < Math.abs(targetTag.id-target))
                    targetTag = detection;
            }

            aprilTagRelocalizationX = field.getAprilTagPose(targetTag.id).position.x - targetTag.pose.z * 39.37 - DISTANCE_FROM_CAMERA_TO_CENTER_X;
            aprilTagRelocalizationY = field.getAprilTagPose(targetTag.id).position.y + targetTag.pose.x * 39.37;
            aprilTagPose = new Pose2d(targetTag.pose.z, targetTag.pose.x, 0);
            driveTrain.pose = new Pose2d(new Vector2d(aprilTagRelocalizationX, aprilTagRelocalizationY), driveTrain.pose.heading);
        }
    }

    public ArrayList<AprilTagDetection> getAprilTagDetections() {
        if (visionOn) {
            if (visionProviderFinalized) {
                return ((AprilTagProvider)visionProviderBack).getDetections();
            }
        }
        return null;
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public void switchVisionProviders() {
        visionProviderBack.shutdownVision();
        if (visionProviderIndex == 2) {
            //switch to AprilTags
            visionProviderIndex = 0;
            visionProviderFinalized = false;

        } else if (visionProviderIndex == 0) {
            //switch back to ColorBlob
            visionProviderIndex = 2;
            visionProviderFinalized = false;
        }
    }

    public void fetchCachedCSPosition() {
        fetchedPosition = positionCache.readPose();
        fetched = fetchedPosition != null;
    }

    public void resetRobotPosFromCache(double loggerTimeoutMinutes, boolean ignoreCache) {
        if(!ignoreCache) {
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
                initPositionTimer = futureTime(1);
                initPositionIndex++;
                break;
            case 1:
                intake.setAngle(Intake.ANGLE_GROUND);
//                if(isPast(initPositionTimer)) {
//                    initPositionTimer = futureTime(1);
//                    initPositionIndex ++;
//                }
                break;
            case 2:
                outtake.slideTargetPosition = Outtake.UNTUCK_SLIDE_POSITION;
//                if (isPast(initPositionTimer)) {
//                    initPositionTimer = futureTime(1);
//                    initPositionIndex ++;
//                }
                break;
            case 3:
                outtake.setTargetAngle(Outtake.FLIPPER_START_ANGLE);
                //                if (isPast(initPositionTimer)) {
//                    initPositionTimer = futureTime(1);
//                    initPositionIndex ++;
//                }
                break;
            case 4:
                outtake.slideTargetPosition = 0;
                break;
            case 5:
                //todo load cached skyhook positions
                //this is the only way to work across power cycles until we incorporate a limit switch calibration
                skyhook.skyhookLeft.setPosition(0);
                skyhook.skyhookRight.setPosition(0);
                skyhook.articulate(Skyhook.Articulation.INIT);
                break;
            case 6:
                intake.articulate(Intake.Articulation.INIT);
        }
    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        switch (this.articulation) {
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
                if (Ingest()) {
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
                if (!(outtake.articulation==Outtake.Articulation.TRAVEL)) {
                    outtake.articulate(Outtake.Articulation.TRAVEL_FROM_INGEST);
                }
                break;
            case TRAVEL_FROM_BACKDROP:
                //assume intake is already in travel
                outtake.articulate(Outtake.Articulation.TRAVEL_FROM_BACKDROP);
                articulation = Articulation.TRAVEL;
                break;
            case BACKDROP_PREP:
                intake.articulate(Intake.Articulation.TRAVEL);
                outtake.articulate(Outtake.Articulation.BACKDROP_PREP);
                articulation = Articulation.BACKDROP;
                break;
        }
        return articulation;
    }

    public void toggleBackdropPrep(){
        if(articulation.equals(Articulation.BACKDROP)){

            articulation = Articulation.TRAVEL_FROM_BACKDROP;
        }
        else{
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
        if(articulation == Articulation.TRAVEL) {
            ingestStage = 0;
            if(intake.articulation == Intake.Articulation.TRAVEL)
                intake.cleanArticulations();
            if(outtake.articulation == Outtake.Articulation.TRAVEL)
                outtake.cleanArticulations();
        }
    }


    public static int ingestStage = 0;
    public long ingestTimer = 0;

    public boolean Ingest() {
        switch (ingestStage) {
            case 0:
                outtake.articulate(Outtake.Articulation.INGEST_FROM_TRAVEL);
                ingestStage++;
            case 1: //wait for outake to dock before proceeding
                if (outtake.articulation == Outtake.Articulation.MANUAL) {
                    //intake can start eating
                    intake.articulate(Intake.Articulation.INGEST);
                    ingestStage++;
                }
                break;
            case 2:  //wait until Intake ingest and swallow are done
                if (!intake.isEating()) ingestStage++;
                break;
            case 3:
                ingestStage=0;
                return true;

        }
        return false;
    }

    public void enterTravel() {
        if(articulation.equals(Articulation.BACKDROP)) {
            articulation = Articulation.TRAVEL_FROM_BACKDROP;
        }
        if(articulation.equals(Articulation.INGEST)) {
            articulation = Articulation.TRAVEL_FROM_INGEST;
        }
        else{
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
        telemetryMap.put("Vision On/Vision Provider Finalized", visionOn+" "+visionProviderFinalized);
        telemetryMap.put("april tag relocalization point", "("+aprilTagRelocalizationX+", "+aprilTagRelocalizationY+")");
        telemetryMap.put("april tag pose", "("+aprilTagPose.position.x+", "+aprilTagPose.position.y+")");
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
            visionProviderBack = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance().setRedAlliance(alliance==Constants.Alliance.RED);
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
