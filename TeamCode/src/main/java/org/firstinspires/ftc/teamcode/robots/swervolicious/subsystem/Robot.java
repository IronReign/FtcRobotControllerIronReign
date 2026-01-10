package org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem;

import static org.firstinspires.ftc.teamcode.robots.deepthought.DriverControls.fieldOrientedDrive;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.field;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
//import org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DTPosition;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.PositionCache;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.Target;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
//import org.firstinspires.ftc.teamcode.robots.swervolicious.Roborama;
import org.firstinspires.ftc.teamcode.robots.swervolicious.Roborama;
import org.firstinspires.ftc.teamcode.robots.swervolicious.rr_localize.SwerveDriveReign;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.samplers.Sampler;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.samplers.SpeciMiner;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;


//@Config(value = "0_ITD_SWERVE")
public class Robot implements Subsystem {

    public static double LED_POWER = .1;
    public static double APRILTAG_Y_OFFSET = 0;
    public static double APRILTAG_X_OFFSET = 0;
    //components and subsystems
    public Subsystem[] subsystems;
    public DriveTrain driveTrain;
    //public TriSwerve driveTrain;
//    public VisionProvider visionProviderBack, visionProviderFront;

    public Limelight3A limelight; // smart camera
    public static boolean updatePositionCache = false;
    public static boolean visionOn = true;

    public boolean calibrating = false;

    public PositionCache positionCache;
    public DTPosition currPosition;

    public DTPosition fetchedPosition;

    public PIDController sampleAlignmentPID;
    public static PIDCoefficients sampleAlignmentCoefficients = new PIDCoefficients(0.03, 0.04, 0);

    public static double SAMPLE_ALIGN_TARGET_TX = 7;
    public static double SAMPLE_ALIGN_TOLERANCE = 3;
    public double PIDError, PIDCorrection;

    //vision variables
    public static boolean visionProviderFinalized = false;
    //REMOVE
    public Pose2d aprilTagPose = new Pose2d(0, 0, 0);

    private long[] subsystemUpdateTimes;
    private List<LynxModule> hubs = null;
    public HardwareMap hardwareMap;
    private VoltageSensor batteryVoltageSensor;
    public Articulation articulation;
    public List<Target> targets = new ArrayList<Target>();
    public boolean fetched;


    public static double PAN_FORWARD = 1970;
    public static double PAN_BASKET_APRILTAG = 1220;
    public static double PAN_SPECIMINER_APRILTAG = 1020;

    public static double panTargetPosition = PAN_BASKET_APRILTAG;

    DcMotor LED;
    public static boolean onTarget;


    public int quickTripStage = 0;
    public long quickTripTimer = 0;

    public static int travelDistance = 5;

    public boolean quickTrip(TelemetryPacket packet) {
        switch (quickTripStage) {
            case 0:
                if (robot.driveTrain.strafeToPose(new Pose2d(new Vector2d(travelDistance, 0), 0), packet)) {
                    quickTripStage++;
                    quickTripTimer = futureTime(5);
                }
                break;
            case 1:
                if (isPast(quickTripTimer)) {
                    quickTripStage++;
                }
                break;
            case 2:
                if (robot.driveTrain.strafeToPose(new Pose2d(new Vector2d(0, 0), 0), packet)) {
                    quickTripStage++;
                }
                break;
            case 3:
            return true;
        }
        return false;
    }

    public enum Articulation {
        MANUAL, SAMPLER_INTAKE, TRAVEL, SAMPLER_OUTTAKE, SPECIMINER_GROUNDTAKE, SPECIMINER_WALLTAKE, SAMPLER_PREP, SPECIMINER_OUTTAKE
    }

    public void start() {

        limelight.start();
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
//            createVisionProviders();

            positionCache = new PositionCache(5);

            // initializing subsystems
            driveTrain = new DriveTrain(hardwareMap, this, false);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");


            subsystems = new Subsystem[]{driveTrain};
            subsystemUpdateTimes = new long[subsystems.length];

            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            sampleAlignmentPID = new PIDController(sampleAlignmentCoefficients);
            articulation = Articulation.MANUAL;
        }

    }
    //end constructor

    public void resetStates() {
        intakeIndex = 0;
        outtakeIndex = 0;
        speciMinerOuttakeIndex = 0;
        walltakeIndex = 0;
        groundtakeIndex = 0;
        for (Subsystem k : subsystems) {
            k.resetStates();
        }
    }

    public double deltaTime = 0;
    long lastTime = 0;

    @Override
    public void update(Canvas fieldOverlay) {
        deltaTime = (System.nanoTime() - lastTime) / 1e9;
        lastTime = System.nanoTime();
        clearBulkCaches(); //ALWAYS FIRST LINE IN UPDATE

        if (updatePositionCache && gameState.isAutonomous()) {
            positionCache.update(currPosition, false);
        }

        LED.setPower(LED_POWER);


//        if (!gameState.isAutonomous)
//            aprilTagRelocalization();
        articulate(articulation);
        // todo decide whether we want to create the pose history:
        //driveTrain.updatePoseEstimate();

        drawRobot(fieldOverlay, driveTrain.localizer.getPose());
        //update subsystems
        for (int i = 0; i < subsystems.length; i++) {
            Subsystem subsystem = subsystems[i];
            long updateStartTime = System.nanoTime();
            subsystem.update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }
    }
    //end update

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 8;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }


    public Articulation articulate(Articulation target) {
        articulation = target;
        switch (this.articulation) {
            case MANUAL:
//            case CALIBRATE:
//                if (calibrate())
                break;
            case SPECIMINER_OUTTAKE:
                break;
        }
        return articulation;
    }


    public int speciMinerOuttakeIndex = 0;


    public int walltakeIndex = 0;


    public int groundtakeIndex = 0;


    public int intakeIndex = 0;


    public int outtakeIndex = 0;


    @Override
    public void stop() {
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
        telemetryMap.put("calibrating", calibrating);
        telemetryMap.put("april tag pose", "(" + aprilTagPose.position.x / 23.5 + ", " + aprilTagPose.position.y / 23.5 + ")");
        telemetryMap.put("MemoryPose", positionCache.readPose());


        telemetryMap.put("onTarget", onTarget);
        telemetryMap.put("pid error", PIDError);
        telemetryMap.put("pid correction", PIDCorrection);
        telemetryMap.put("limelight running?", limelight.isRunning());
        LLStatus status = limelight.getStatus();
        telemetryMap.put("limelight fps, ", status.getFps());
        telemetryMap.put("limelight pipeline", "index: " + status.getPipelineIndex() + " type: " + status.getPipelineType());
        //todo - remove unnecessary telemetry here
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            telemetryMap.put("limelight botpose", result.getBotpose());
            telemetryMap.put("# of limelight detections", result.getDetectorResults().size());
            telemetryMap.put("limelight tx", result.getTx());
        }
        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
        }


        telemetryMap.put("Delta Time", deltaTime);


        return telemetryMap;
    }
    //end getTelemetry

//    public void createVisionProviders() {
//        try {
//            visionProviderBack = VisionProviders.VISION_PROVIDERS[backVisionProviderIndex].newInstance().setRedAlliance(alliance.isRed());
//            visionProviderFront = VisionProviders.VISION_PROVIDERS[frontVisionProviderIndex].newInstance().setRedAlliance(alliance.isRed());
//        } catch (IllegalAccessException | InstantiationException e) {
//            throw new RuntimeException("Error while instantiating vision provider");
//        }
//    }

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

    public void fetchCachedDTPosition() {
        fetchedPosition = positionCache.readPose();
        fetched = fetchedPosition != null;
    }


    public int alignOnSampleState = 0;

    public boolean alignOnSample() {

        //        switch (alignOnSampleState) {
//            case 0:
        limelight.pipelineSwitch(3);
        LLResult llResult;
        panTargetPosition = PAN_FORWARD;

//                break;
//            case 1:
        if ((llResult = limelight.getLatestResult()) != null) {
            if (llResult.getTx() != 0.0) {
                double targetTx = SAMPLE_ALIGN_TARGET_TX;
                sampleAlignmentPID.setPID(sampleAlignmentCoefficients);
                sampleAlignmentPID.setInputRange(-25, 25);
                sampleAlignmentPID.setInput(llResult.getTx());
                sampleAlignmentPID.setSetpoint(targetTx);
                sampleAlignmentPID.setOutputRange(-.7, .7);
                sampleAlignmentPID.setTolerance(SAMPLE_ALIGN_TOLERANCE);
                double correction = sampleAlignmentPID.performPID();
                PIDCorrection = correction;
                PIDError = sampleAlignmentPID.getError();
                sampleAlignmentPID.enable();
                if (sampleAlignmentPID.onTarget()) {
                    onTarget = true;
                    sampleAlignmentPID.clearCache();
                    driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    return true;

                } else {
                    onTarget = false;
                    driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), correction));
                    return false;
                }
            } else {
                driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
        } else {
            driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        }
//                break;
//        }
//        driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//
//        return true;
//
        return false;
    }

    //to be called repeatedly until success
    public void aprilTagRelocalization(boolean forward) {
        if (forward) {
            limelight.pipelineSwitch(5);
            panTargetPosition = PAN_FORWARD;
            LLResult llResult;
            if ((llResult = limelight.getLatestResult()) != null) {
                //limelight returns everything in meters
                aprilTagPose = new Pose2d(new Vector2d(llResult.getBotpose().getPosition().x * 39.37, llResult.getBotpose().getPosition().y * 39.37), llResult.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
                if (llResult.getBotpose().getPosition().x != 0) ;
//                    driveTrain.setPose(aprilTagPose);
            }
        } else {
            limelight.pipelineSwitch(2);
            panTargetPosition = PAN_BASKET_APRILTAG;
        }
        LLResult llResult;
        if ((llResult = limelight.getLatestResult()) != null) {
            //limelight returns everything in meters
            aprilTagPose = new Pose2d(new Vector2d(llResult.getBotpose().getPosition().x * 39.37, llResult.getBotpose().getPosition().y * 39.37), llResult.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
            if (llResult.getBotpose().getPosition().x != 0) ;
//                driveTrain.setPose(aprilTagPose);
        }
    }


//    public void enableVision() {
//        if (visionOn) {
//            if (!visionProviderFinalized) {
//                createVisionProviders();
//                visionProviderBack.initializeVision(hardwareMap, this, false);
//                visionProviderFront.initializeVision(hardwareMap, this, true);
//                visionProviderFinalized = true;
////
//            }

    /// /            if (frontVision) {
    /// /                visionProviderFront.update(debugTelemetryEnabled);
    /// /            } else visionProviderBack.update(debugTelemetryEnabled);
//        }
//    }


//    public ArrayList<AprilTagDetection> getAprilTagDetections() {
//        if (visionOn) {
//            if (visionProviderFinalized) {
//                return ((AprilTagProvider) visionProviderBack).getDetections();
//            }
//        }
//        return null;
//    }

//    public void switchVisionProviders() {
//        visionProviderBack.shutdownVision();
//        visionProviderFront.shutdownVision();
//        if (backVisionProviderIndex == 2) {
//            //switch to AprilTags
//            backVisionProviderIndex = 0;
//            visionProviderFinalized = false;
//
//        } else if (backVisionProviderIndex == 0) {
//            //switch back to ColorBlob
//            backVisionProviderIndex = 2;
//            visionProviderFinalized = false;
//        }
//    }
    public boolean calibrate() {
        return driveTrain.calibrate();
        // todo this can only be called if trident calibrationIndex started at zero
    }

}
