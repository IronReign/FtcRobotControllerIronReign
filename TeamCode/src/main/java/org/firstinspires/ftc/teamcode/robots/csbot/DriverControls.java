package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.active;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.visionProviderIndex;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.juiceDriveTrain;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;

public class DriverControls {
    //CONSTANTS
    public static double TURRET_DEADZONE = 0.03;

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
        if(stickyGamepad1.left_stick_button) {
            robot.createVisionProvider();
        }
        if(!gamepad1.atRest()) {
            telemetry.addData("gamepad1", "not zeroed");
        }
        if(!gamepad2.atRest()) {
            telemetry.addData("gamepad2", "not zeroed");
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

    public void joystickDrive() {
        if (gamepad1.right_trigger > .1) {
            robot.intake.adjustBeaterBarAngle(gamepad1.right_trigger);
        }
        if (gamepad1.left_trigger > .1) {
            robot.intake.adjustBeaterBarAngle(-gamepad1.left_trigger);
        }
        if (stickyGamepad1.a) {
            robot.intake.toggleBeaterBar();
        }
        if (stickyGamepad1.b) {
            robot.intake.switchBeaterBarDirection();
        }
        if(gamepad1.dpad_right)
            robot.driveTrain.line();
        if (!juiceDriveTrain)
           robot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        else
            robot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (stickyGamepad1.right_bumper)
            robot.outtake.moveSlide(50);
        if (stickyGamepad1.left_bumper)
            robot.outtake.moveSlide(-50);

        if (stickyGamepad1.y)
            robot.outtake.raiseFlipper(10);
        if (stickyGamepad1.x)
            robot.outtake.lowerFlipper(10);

        if (stickyGamepad1.dpad_down) {
            juiceDriveTrain = !juiceDriveTrain;
        }
//        if(stickyGamepad1.dpad_right){
//            robot.outtake.clearFlipper();
//        }
//        if(stickyGamepad1.dpad_up) {
//            robot.outtake.scoreFlipper();
//        }
//        if(stickyGamepad1.dpad_left) {
//            robot.outtake.tuckFlipper();
//        }

        if (stickyGamepad1.guide) {
            robot.switchVisionProviders();
        }

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

        if (stickyGamepad1.dpad_down) {
            robot.articulate(Robot.Articulation.CALIBRATE);
        }
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
