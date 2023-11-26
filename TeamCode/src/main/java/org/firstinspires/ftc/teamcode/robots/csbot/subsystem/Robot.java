package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.gameState;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Target;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;


@Config(value = "AA_CSRobot")
public class Robot implements Subsystem {

    //components and subsystems
    public Subsystem[] subsystems;
    public CSDriveTrain driveTrain;
    public Intake intake;
    public VisionProvider visionProviderBack = null;
    public static boolean visionOn = true;
    public Outtake outtake;
    //TODO - create a field
//    public Field field;

    //vision variables
    public boolean visionProviderFinalized = false;
    public static int visionProviderIndex = 2;
    public static boolean colorBlobEnabled = true;


    private long[] subsystemUpdateTimes;
    private final List<LynxModule> hubs;
    public static boolean juiceDriveTrain = false;
    public HardwareMap hardwareMap;
    private VoltageSensor batteryVoltageSensor;
    private Articulation articulation;
    public List<Target> targets = new ArrayList<Target>();

    public enum Articulation {
        //beater bar, drivetrain, drone launcher, outtake
        MANUAL,
        AUTON,
        CALIBRATE,
        SCORE_PIXEL,
        INTAKE_PIXEL,
        FOLD,
        WING_INTAKE,
        UNFOLD,
        HANG,
        LAUNCH_DRONE,

    }

    public void  start() {
        //TODO - articulate starting position
        if(gameState.isAutonomous()) {
            intake.setAngleControllerTicks(1600);
        }
        articulation = Articulation.MANUAL;
    }
    //end start


    public Robot(HardwareMap hardwareMap, boolean simulated) {
        this.hardwareMap = hardwareMap;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        //initialize vision
        createVisionProvider();


        // initializing subsystems
        driveTrain = new CSDriveTrain(hardwareMap, this, simulated);
        //TODO - THIS IS FOR MANUAL ONLY
        intake = new Intake(hardwareMap, this);
        outtake = new Outtake(hardwareMap, this);

        subsystems = new Subsystem[]{driveTrain, intake, outtake}; //{driveTrain, turret, crane};
        subsystemUpdateTimes = new long[subsystems.length];

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        articulation = Robot.Articulation.MANUAL;

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

    public void updateVision() {
        if(visionOn) {
            if (!visionProviderFinalized) {
                createVisionProvider();
                visionProviderBack.initializeVision(hardwareMap, this);
                visionProviderFinalized = true;

            }
            visionProviderBack.update();
        }
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
        if(visionProviderIndex == 2){
            //switch to AprilTags
            visionProviderIndex = 0;
            visionProviderFinalized = false;

        }
        else if (visionProviderIndex == 0) {
            //switch back to ColorBlob
            visionProviderIndex = 2;
            visionProviderFinalized = false;
        }
    }



    public static int initPositionIndex = 0;
    public long initPositionTimer;
    public void initPosition() {
        switch (initPositionIndex) {
            case 0:
                initPositionTimer = futureTime(1);
                initPositionIndex ++;
                break;
            case 1:
                intake.articulate(Intake.Articulation.FOLD);
//                if(isPast(initPositionTimer)) {
//                    initPositionTimer = futureTime(1);
//                    initPositionIndex ++;
//                }
                break;
            case 2:
                outtake.slidePosition = Outtake.UNTUCK_SLIDE_POSITION;
//                if (isPast(initPositionTimer)) {
//                    initPositionTimer = futureTime(1);
//                    initPositionIndex ++;
//                }
                break;
            case 3:
                Outtake.flipperPosition = Outtake.FLIPPER_INIT_POSITION;
//                if (isPast(initPositionTimer)) {
//                    initPositionTimer = futureTime(1);
//                    initPositionIndex ++;
//                }
                break;
            case 4:
                outtake.slidePosition = 0;
                break;
            case 5:
                break;
        }
    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        switch (this.articulation) {
            case MANUAL:
                break;
            case CALIBRATE:
                //TODO - WRITE A CALIBRATION ROUTINE
                break;
            case WING_INTAKE:
                if (wingIntake()) {
                    articulation = Articulation.MANUAL;
                }
                break;

        }
        return articulation;
    }

    @Override
    public void stop() {
        for (Subsystem component : subsystems) {
            component.stop();
        }
    }
    //end stop



    public static int wingIntakeIndex = 0;
    public long wingIntakeTimer = 0;
    public boolean wingIntake () {
        switch (wingIntakeIndex) {
            case 0:
                wingIntakeTimer = futureTime(.5);
                intake.articulate(Intake.Articulation.WING_INTAKE_POSTION);
                if (intake.articulation == Intake.Articulation.MANUAL && isPast(wingIntakeTimer))
                    wingIntakeIndex ++;
                break;
            case 1:
                outtake.articulate(Outtake.Articulation.INTAKE_PIXEL);
                if(outtake.articulation == Outtake.Articulation.MANUAL) {
                    wingIntakeIndex ++;
                }
                break;
            case 2:
                return true;

        }
        return false;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);
        telemetryMap.put("wingIntakeIndex", wingIntakeIndex);
        telemetryMap.put("initPositionIndex", initPositionIndex);
        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
        }

        if (debug) {
            telemetryMap.put("driveTrain juiced?", juiceDriveTrain);
        }

        telemetryMap.put("Delta Time", deltaTime);


        return telemetryMap;
    }
    //end getTelemetry

    public void createVisionProvider() {
        try {
            visionProviderBack = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance().setRedAlliance(alliance.getMod());
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
