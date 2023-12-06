package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.active;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.juiceDriveTrain;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.visionOn;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.visionProviderIndex;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;

public class DriverControls {
    public static double PRECISION_TURN_MULTIPLIER = .8;
    public static double PRECISION_DRIVE_MULTIPLIER = .8;
    //CONSTANTS
    public static double DEADZONE = 0.1;

    public boolean visionProviderFinalized = robot.visionProviderFinalized;

    Gamepad gamepad1, gamepad2;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    DriverControls(Gamepad pad1, Gamepad pad2) {
        gamepad1 = pad1;
        gamepad2 = pad2;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
    }

    public void init_loop() {
        if (stickyGamepad1.left_stick_button) {
            robot.createVisionProvider();
        }
        if(stickyGamepad1.a) {
            CenterStage_6832.initPosition = true;
        }

        if(stickyGamepad1.guide) {
            robot.initPositionIndex ++;
        }
        updateStickyGamepads();
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
            robot.outtake.flipperTest();
        }

        if (stickyGamepad1.guide) {
            robot.switchVisionProviders();
        }

        if(stickyGamepad1.left_stick_button) {
            visionOn = !visionOn;
        }

        if(stickyGamepad1.back) {
            robot.articulate(Robot.Articulation.WING_INTAKE);
        }

        fieldOrientedDrive();

    }

    public void joystickDrive() {
        if (gamepad1.left_trigger > .1) {
            robot.intake.adjustBeaterBarAngle(gamepad1.right_trigger*.85);
        }
        if (gamepad1.right_trigger > .1) {
            robot.intake.adjustBeaterBarAngle(-gamepad1.left_trigger*.85);
        }
        if (stickyGamepad1.a) {
            robot.intake.toggleBeaterBar();
        }
        if (stickyGamepad1.b) {
            robot.intake.switchBeaterBarDirection();
        }

        fieldOrientedDrive();

        if (gamepad1.left_bumper)
            robot.outtake.moveSlide(5);
        if (gamepad1.right_bumper)
            robot.outtake.moveSlide(-5);

        if (gamepad1.y)
            robot.outtake.adjustFlipper(-5);
        if (gamepad1.x)
            robot.outtake.adjustFlipper( 5);

        if(stickyGamepad1.dpad_up)
            robot.intake.articulate(Intake.Articulation.SWALLOW);

        if(stickyGamepad1.dpad_left) {
            robot.skyhook.releaseTheJimmy();
        }

        if (stickyGamepad1.dpad_down) {
            juiceDriveTrain = !juiceDriveTrain;
        }

        //mu name is jimmy and i am a pickle and i am a potato and i need to sleep with my truffle oil to feel happiness

    }

    public void fieldOrientedDrive() {
        if(Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                Math.abs(gamepad1.right_stick_x ) > DEADZONE) {
            robot.driveTrain.fieldOrientedDrive(alliance == Constants.Alliance.BLUE? -gamepad1.left_stick_y: gamepad1.left_stick_y, alliance == Constants.Alliance.BLUE? -gamepad1.left_stick_x : gamepad1.left_stick_x,  -gamepad1.right_stick_x);

        }
        else robot.driveTrain.fieldOrientedDrive(0, 0, 0);
    }



    public void handleStateSwitch() {
        if (!active) {
            if (stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
                CenterStage_6832.gameStateIndex -= 1;
            if (stickyGamepad1.right_bumper || stickyGamepad2.right_bumper)
                CenterStage_6832.gameStateIndex += 1;
            if (CenterStage_6832.gameStateIndex < 0)
                CenterStage_6832.gameStateIndex = CenterStage_6832.GameState.getNumGameStates() - 1;
            CenterStage_6832.gameStateIndex %= CenterStage_6832.GameState.getNumGameStates();
            CenterStage_6832.gameState = CenterStage_6832.GameState.getGameState(CenterStage_6832.gameStateIndex);
        }

        if (stickyGamepad1.back || stickyGamepad2.back)
            active = !active;


    }

    void handlePregameControls() {
        if (stickyGamepad1.x || stickyGamepad2.x) {
            alliance = Constants.Alliance.BLUE;
            startingPosition = startingPosition.getMod() == false ?
                    startingPosition :
                        startingPosition == Constants.Position.START_LEFT_RED ?
                            Constants.Position.START_LEFT_BLUE : Constants.Position.START_RIGHT_BLUE;

        }
        if (stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Constants.Alliance.RED;
            startingPosition = startingPosition.getMod() == true ?
                    startingPosition :
                    startingPosition == Constants.Position.START_LEFT_BLUE ?
                            Constants.Position.START_LEFT_RED : Constants.Position.START_RIGHT_RED;
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
                    robot.visionProviderBack.initializeVision(hardwareMap); // this is blocking
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
