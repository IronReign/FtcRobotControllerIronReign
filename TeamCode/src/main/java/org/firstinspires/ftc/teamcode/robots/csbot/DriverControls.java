package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.active;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.visionOn;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.visionProviderIndex;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Skyhook;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;

public class DriverControls {
    public static double PRECISION_TURN_MULTIPLIER = .8;
    public static double PRECISION_DRIVE_MULTIPLIER = .8;
    //CONSTANTS
    public static boolean fieldOrientedDrive = true;
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
        updateStickyGamepads();
        if (stickyGamepad1.left_stick_button) {
            robot.createVisionProvider();
        }

        if(stickyGamepad1.dpad_up) {
            robot.visionProviderFinalized = !robot.visionProviderFinalized;
        }

        if(stickyGamepad1.a) {
            CenterStage_6832.initPosition = true;
        }


        if(stickyGamepad1.guide) {
            robot.initPositionIndex ++;
        }

        if(stickyGamepad2.a) {
            robot.skyhook.articulate(Skyhook.Articulation.INIT);
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
            robot.outtake.flipperTest();
        }

        if (stickyGamepad1.guide) {
            robot.switchVisionProviders();
        }

        if(stickyGamepad1.left_stick_button) {
            visionOn = !visionOn;
        }

        if(stickyGamepad1.back) {
            robot.articulate(Robot.Articulation.INGEST);
        }

        fieldOrientedDrive();

    }

    public void joystickDrive() {

        if(stickyGamepad2.b) {
            fieldOrientedDrive = !fieldOrientedDrive;
        }
        if (gamepad1.left_trigger > .1) {
            robot.intake.adjustAngle(gamepad1.left_trigger*.85);
        }
        if (gamepad1.right_trigger > .1) {
            robot.intake.adjustAngle(-gamepad1.right_trigger*.85);
        }
        if (stickyGamepad1.a) {
            robot.intake.articulate(Intake.Articulation.INGEST);
        }
        if (stickyGamepad1.b) {
            robot.intake.toggleBeaterDirection();
        }
        if (stickyGamepad1.y){
            robot.articulate(Robot.Articulation.INGEST);
        }

        if(fieldOrientedDrive) {
            fieldOrientedDrive();
        }
        else {
            robotOrientedDrive();
        }

        if (gamepad1.left_bumper) {
            if (robot.intake.isEating())
                robot.intake.pixelSensorLeft();
            else
                robot.outtake.moveSlide(5);
//            if(robot.outtake.getSlidePosition() > 100 && robot.outtake.getSlidePosition() < 800)//TODO find more accurate values for where flipper should be raised
//                robot.outtake.setTargetAngle(Outtake.FLIPPER_START_ANGLE);
        }
        if (gamepad1.right_bumper) {
            if (robot.intake.isEating())
                robot.intake.pixelSensorRight();
            else
                robot.outtake.moveSlide(-5);
//            if(robot.outtake.getSlidePosition() > 100 && robot.outtake.getSlidePosition() < 800)//TODO find more accurate values for where flipper should be raised
//                robot.outtake.setTargetAngle(Outtake.FLIPPER_START_ANGLE);
        }

        if (gamepad1.y)
            robot.outtake.adjustFlipper(-15);
        if (gamepad1.x)
            robot.outtake.adjustFlipper(15);

        if(stickyGamepad1.dpad_up)
            robot.intake.articulate(Intake.Articulation.SWALLOW);

//        if(stickyGamepad1.dpad_left) {
//            robot.skyhook.releaseTheJimmy();
//        }

        if(stickyGamepad2.y) {
            fieldOrientedDrive = !fieldOrientedDrive;
        }
        if (stickyGamepad1.dpad_down) {
            robot.outtake.articulate(Outtake.Articulation.SCORE_PIXEL);
        }

        if(stickyGamepad2.dpad_up) {
            robot.skyhook.articulate(Skyhook.Articulation.PREP_FOR_HANG);
        }
        if(stickyGamepad2.a) {
            robot.skyhook.articulate(Skyhook.Articulation.INIT);
        }

        if(stickyGamepad2.dpad_down) {
            robot.articulate(Robot.Articulation.HANG);
        }
//        if(stickyGamepad2.dpad_up) {
//            robot.articulate(Robot.Articulation.LAUNCH_DRONE);
//        }

        //mu name is jimmy and i am a pickle and i am a potato and i need to sleep with my truffle oil to feel happiness

    }

    public void fieldOrientedDrive() {
        if(Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                Math.abs(gamepad1.right_stick_x ) > DEADZONE) {
            robot.driveTrain.fieldOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, alliance.getMod());
        }
        else robot.driveTrain.drive(0, 0, 0);
    }

    public void robotOrientedDrive() {
        robot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
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
