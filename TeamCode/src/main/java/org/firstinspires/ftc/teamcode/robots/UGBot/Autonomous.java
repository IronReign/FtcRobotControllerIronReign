package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.SkystoneVisionProvider;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.VisionProvider;

import static org.firstinspires.ftc.teamcode.util.Conversions.servoNormalize;

/**
 * Class to keep all autonomous-related functions and state-machines in
 */
public class Autonomous {

    private PoseUG robot;
    private Telemetry telemetry;
    private Gamepad gamepad1;

    public static int sampleExtendMiddle = 2210;
    public static int sampleExtendLeft = 2200;
    public static int sampleExtendRight = 2200;
    public static boolean sampleContinue = false;

    // vision-related configuration
    public VisionProvider vp;
    public int visionProviderState = 2;
    public boolean visionProviderFinalized;
    public boolean enableTelemetry = true;
    public static final Class<? extends VisionProvider>[] visionProviders = VisionProviders.visionProviders;
    public static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    public int ugState = 0;
    private MineralStateProvider ugStateProvider = () -> ugState;

    // staging and timer variables
    public float autoDelay = 0;
    public Stage autoStage = new Stage();
    public Stage autoSetupStage = new Stage();

    // auto constants
    private static final double DRIVE_POWER = .65;
    private static final float TURN_TIME = 2;
    private static final float DUCKY_TIME = 1.0f;

    public Autonomous(PoseUG robot, Telemetry telemetry, Gamepad gamepad1) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }

    public boolean sample() {
        // Turn on camera to see which is gold
        StackHeight gp = vp.detect();
        // Hold state lets us know that we haven't finished looping through detection
        if (gp != StackHeight.NONE_FOUND) {
            switch (gp) {
                case ZERO:
                    ugState = 0;
                    break;
                case FOUR:
                    ugState = 2;
                    break;
                case ONE:
                case NONE_FOUND:
                default:
                    ugState = 1;
                    break;
            }
            telemetry.addData("Vision Detection", "Stack Height: %s", gp.toString());
            vp.shutdownVision();
            return true;
        } else {
            telemetry.addData("Vision Detection", "NONE_FOUND (still looping through internally)");
            return false;
        }
    }

    public StateMachine simultaneousStateTest = getStateMachine(autoStage).addSimultaneousStates(() -> {
        robot.turret.rotateRight(0.25);
        return false;
    }, () -> {
        robot.driveMixerDiffSteer(0.25, 0.25);
        return false;
    }).build();


    public StateMachine AutoFull = getStateMachine(autoStage)
//            .addTimedState(3f, () -> sample(),  () -> telemetry.addData("DELAY", "DONE"))

            .addMineralState(ugStateProvider,
                    () -> robot.driveGenericPIDDistance(.5,-0.4572*50, -robot.getDistRightDist()*50,true, 3),
                    () -> robot.driveGenericPIDDistance(.5,-0.4572*50, -robot.getDistRightDist()*50,true, 2.3), // 2.3
                    () -> robot.driveGenericPIDDistance(.5,-0.4572*50, -robot.getDistRightDist()*50,true, 1.87)) //1.87

            .addMineralState(ugStateProvider,
                    () -> robot.turret.rotateCardinalTurret(true),
                    () -> robot.turret.rotateCardinalTurret(false),
                    () -> robot.turret.rotateCardinalTurret(true))

            .addMineralState(ugStateProvider,
                    () -> robot.rotateIMU(270,3),
                    () -> robot.rotateIMU(90,3),
                    () -> robot.rotateIMU(270,3))

            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            //.addState(() -> robot.launcher.toggleGripper())
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addState(() -> robot.rotateIMU(0,3))

            .addMineralState(ugStateProvider,
                    () -> robot.driveIMUDistanceWithReset(.7, 0, false, 1), //1, backwards
                    () -> robot.driveIMUDistanceWithReset(.7, 0, false, .5), // .5
                    () -> true)
            .addState(() -> robot.driveIMUDistanceWithReset(.2,180,true,0))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .build();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // //
    // Old Autonomous Routines //
    // //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private StateMachine.Builder getStateMachine(Stage stage) {
//        return
//                StateMachine.builder()
//                        .stateSwitchAction(() -> robot.launcher.setGripperPos(robot.launcher.toggleGripper()))
//                        .stateEndAction(() -> robot.turret.maintainHeadingTurret(false)).stage(stage);

            return StateMachine.builder()
                    .stateSwitchAction(() -> robot.resetMotors(true))
                    .stateEndAction(() -> {})
                    .stage(stage);

    }

    public void deinitVisionProvider() {
        telemetry.addData("Please wait", "Deinitializing vision");
        // telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        vp.shutdownVision();
        vp = null;
        visionProviderFinalized = false;
    }

    public void initVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            // telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = visionProviders[visionProviderState].newInstance();
             vp.initializeVision(robot.hwMap, viewpoint, this.enableTelemetry);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    public void initDummyVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            // telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = VisionProviders.defaultProvider.newInstance();
             vp.initializeVision(robot.hwMap, viewpoint, false);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }
}