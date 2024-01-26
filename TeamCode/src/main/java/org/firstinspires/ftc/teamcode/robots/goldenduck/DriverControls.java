package org.firstinspires.ftc.teamcode.robots.goldenduck;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem.Robot;

import static org.firstinspires.ftc.teamcode.robots.goldenduck.CoreTele.active;
import static org.firstinspires.ftc.teamcode.robots.goldenduck.CoreTele.alliance;
import static org.firstinspires.ftc.teamcode.robots.goldenduck.CoreTele.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.goldenduck.CoreTele.robot;
import static org.firstinspires.ftc.teamcode.robots.goldenduck.CoreTele.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem.Robot.visionOn;
import static org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem.Robot.visionProviderIndex;

public class DriverControls {
    public static double PRECISION_TURN_MULTIPLIER = .8;
    public static double PRECISION_DRIVE_MULTIPLIER = .8;
    //CONSTANTS
    public static boolean fieldOrientedDrive = false;
    public static double DEADZONE = 0.1;

    public boolean visionProviderFinalized = robot.visionProviderFinalized;
    boolean droneSet = true;

    Gamepad gamepad1, gamepad2;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    DriverControls(Gamepad pad1, Gamepad pad2) {
        gamepad1 = pad1;
        gamepad2 = pad2;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
    }

    public void init_loop() {
        updateStickyGamepads();
        if (stickyGamepad1.left_stick_button) {
            robot.createVisionProvider();
        }

        if(stickyGamepad1.dpad_up) {
            robot.visionProviderFinalized = !robot.visionProviderFinalized;
        }

        if(stickyGamepad1.a) {
            CoreTele.initPosition = true;
        }

        if (stickyGamepad1.y)
            robot.arm.GripOuterToggle();
        //robot.arm.GripNeither();

        if (stickyGamepad1.x)
            robot.arm.GripInnerToggle();
        //robot.arm.GripBoth();


        if(stickyGamepad1.guide) {
            robot.initPositionIndex ++;
        }

        if(stickyGamepad2.a) {

        }


        handleStateSwitch();
        handlePregameControls();
//        handleVisionProviderSwitch();
    }

    public void updateStickyGamepads() {
        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    public void manualDiagnosticMethods() {
        if(gamepad1.right_bumper) {

        }

        if (stickyGamepad1.guide) {
            robot.switchVisionProviders();
        }

        if(stickyGamepad1.left_stick_button) {
            visionOn = !visionOn;
        }

        fieldOrientedDrive();

    }

    public void joystickDrive() {

        if(stickyGamepad2.b) {
            //fieldOrientedDrive = !fieldOrientedDrive;
        }
        if (gamepad1.left_trigger > .1) {
//            robot.arm.adjustShoulder(-gamepad1.left_trigger);
            robot.skyhooks.adjustSkyHooks(-gamepad1.left_trigger);
        }

        if (gamepad1.right_trigger > .1) {
//            robot.arm.adjustShoulder(gamepad1.right_trigger);
            robot.skyhooks.adjustSkyHooks(gamepad1.right_trigger);

        }
        if (stickyGamepad1.a) {
            //ask the robot to snap to an ingest config
            //todo make this behavior work for core's robot
            //robot.behave(Robot.Behavior.INGEST);
            robot.arm.WristPickup();
        }
        if (stickyGamepad1.b) {
            //ask the robot to configure for scoring at backdrop
            //todo make this behavior work for core's robot
            //robot.toggleBackdropPrep();
            robot.arm.WristBackdrop();
        }

        if(fieldOrientedDrive) {
            fieldOrientedDrive();
        }
        else {
            robotOrientedDrive();
        }

        if (gamepad1.left_bumper) {

        }
        if (stickyGamepad1.right_bumper) {
            //cycle between Arm presets: Intake, Travel, Backdrop
            robot.arm.armToggle();

        }

        if (stickyGamepad1.y)
            robot.arm.GripOuterToggle();
            //robot.arm.GripNeither();

        if (stickyGamepad1.x)
            robot.arm.GripInnerToggle();
            //robot.arm.GripBoth();


        if(stickyGamepad1.dpad_up)
            if(droneSet)
            {
                robot.arm.droneLaunch();
                droneSet=false;
            }
        else {
            robot.arm.droneSet();
            droneSet=true;
            }

        if(stickyGamepad2.y) {
            //fieldOrientedDrive = !fieldOrientedDrive;
        }
        if (stickyGamepad1.dpad_down) {
            robot.arm.setDroneShoulderTargetPosition();//drone launch angle
        }

        if(stickyGamepad2.dpad_up) {
            robot.behave(Robot.Behavior.PREP_FOR_HANG);
        }

        if(stickyGamepad2.dpad_down) {
            robot.behave(Robot.Behavior.HANG);
        }

    }

    public void fieldOrientedDrive() {
        if(Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                Math.abs(gamepad1.right_stick_x ) > DEADZONE)
        {
            robot.driveTrain.setHumanIsDriving(true);
            robot.driveTrain.fieldOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, alliance.getMod());
        }
        //stop if no input
        else if (robot.driveTrain.isHumanIsDriving()) robot.driveTrain.drive(0, 0, 0);
    }

    public void robotOrientedDrive() {
        if (Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                Math.abs(gamepad1.right_stick_x) > DEADZONE) {
            robot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        }
        else if (robot.driveTrain.isHumanIsDriving()) robot.driveTrain.drive(0, 0, 0);
    }


    public void handleStateSwitch() {
        if (!active) {
            if (stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
                CoreTele.gameStateIndex -= 1;
            if (stickyGamepad1.right_bumper || stickyGamepad2.right_bumper)
                CoreTele.gameStateIndex += 1;
            if (CoreTele.gameStateIndex < 0)
                CoreTele.gameStateIndex = CoreTele.GameState.getNumGameStates() - 1;
            CoreTele.gameStateIndex %= CoreTele.GameState.getNumGameStates();
            CoreTele.gameState = CoreTele.GameState.getGameState(CoreTele.gameStateIndex);
        }

        if (stickyGamepad1.back || stickyGamepad2.back)
            active = !active;


    }

    @SuppressLint("SuspiciousIndentation")
    void handlePregameControls() {
        if (stickyGamepad1.x || stickyGamepad2.x) {

            alliance = Constants.Alliance.BLUE;

            startingPosition = startingPosition.getMod() == false ?
                    startingPosition :
                        startingPosition == Constants.Position.START_LEFT_RED ?
                            Constants.Position.START_LEFT_BLUE : Constants.Position.START_RIGHT_BLUE;
                robot.visionProviderBack.setRedAlliance(false);
//            robot.visionProviderBack.initializeVision(hardwareMap); // this is blocking


        }
        if (stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Constants.Alliance.RED;
            startingPosition = startingPosition.getMod() == true ?
                    startingPosition :
                    startingPosition == Constants.Position.START_LEFT_BLUE ?
                            Constants.Position.START_LEFT_RED : Constants.Position.START_RIGHT_RED;
                robot.visionProviderBack.setRedAlliance(true);
//            robot.visionProviderBack.initializeVision(hardwareMap); // this is blocking

        }


        if (stickyGamepad1.dpad_left || stickyGamepad2.dpad_left)
            startingPosition = alliance == Constants.Alliance.RED ? Constants.Position.START_LEFT_RED : Constants.Position.START_LEFT_BLUE;

        if (stickyGamepad1.dpad_right || stickyGamepad2.dpad_right)
            startingPosition = alliance == Constants.Alliance.RED ? Constants.Position.START_RIGHT_RED : Constants.Position.START_RIGHT_BLUE;

        if (stickyGamepad1.dpad_up || stickyGamepad2.dpad_up)
            debugTelemetryEnabled = !debugTelemetryEnabled;
    }

    public void handleVisionProviderSwitch() {
        if (!active) {
            if (!visionProviderFinalized) {
                if (stickyGamepad1.dpad_left || stickyGamepad2.dpad_left) {
                    visionProviderIndex = (visionProviderIndex + 1) % VisionProviders.VISION_PROVIDERS.length; // switch vision provider
                    robot.createVisionProvider();
                }
                if (stickyGamepad1.dpad_up || stickyGamepad2.dpad_up) {
//                    robot.visionProviderBack.initializeVision(hardwareMap); // this is blocking
                    visionProviderFinalized = true;
                }
            } else if (stickyGamepad1.dpad_up || stickyGamepad2.dpad_up) {
                robot.visionProviderBack.shutdownVision(); // also blocking, but should be very quick
                visionProviderFinalized = false;
            }
        } else if ((stickyGamepad1.dpad_right || stickyGamepad2.dpad_right) && visionProviderFinalized) {
            robot.visionProviderBack.saveDashboardImage();
        }
        if (visionProviderFinalized)
            robot.visionProviderBack.update();
    }


}
