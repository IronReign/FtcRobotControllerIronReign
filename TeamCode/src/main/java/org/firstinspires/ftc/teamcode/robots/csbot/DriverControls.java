package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.active;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.field;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.visionOn;
import static org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot.visionProviderIndex;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Skyhook;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;

public class DriverControls {
    //CONSTANTS
    public static boolean fieldOrientedDrive = true;
    public static double DEADZONE = 0.1;

    public boolean visionProviderFinalized = robot.visionProviderFinalized;

    Gamepad gamepad1, gamepad2;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    DriverControls(Gamepad pad1, Gamepad pad2) {
        fieldOrientedDrive = true;
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

        if(stickyGamepad1.y) {
            if(gameState.isAutonomous()) {
                robot.driveTrain.setPose(startingPosition);
            }
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

        fieldOrientedDrive();

    }

    public void joystickDrive() {

        //GAMEPAD 1 CONTROLS
        // ------------------------------------------------------------------
        if (gamepad1.left_trigger > .1) {
            robot.outtake.adjustFlipper(-robot.outtake.FLIPPER_ADJUST_ANGLE);
        }
        if (gamepad1.right_trigger > .1) {
            robot.outtake.adjustFlipper(robot.outtake.FLIPPER_ADJUST_ANGLE);
        }
        if (stickyGamepad1.a) {
            if(robot.articulation == Robot.Articulation.TRAVEL)
                robot.articulate(Robot.Articulation.INGEST);
        }
        if (stickyGamepad1.b) {
            robot.toggleBackdropPrep();
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
        }
        if (gamepad1.right_bumper) {
            if (robot.intake.isEating())
                robot.intake.pixelSensorRight();
            else
                robot.outtake.moveSlide(-5);
        }
        
        if(stickyGamepad1.y) {
            if(robot.skyhook.articulation.equals(Skyhook.Articulation.PREP_FOR_HANG)) {
                robot.articulate(Robot.Articulation.HANG);
            }
            else{

                if(CenterStage_6832.totalRunTime > 110/*field.getZone(robot.driveTrain.pose) != null && field.getZone(robot.driveTrain.pose).name.equals("AUDIENCE")*/) {
                    if(robot.intake.getIngestPixelHeight() != 4)
                        robot.intake.setIngestPixelHeight(4);
                    else if(robot.intake.getIngestPixelHeight() == 4)
                        robot.intake.setIngestPixelHeight(0);
                }
                else {
                    robot.articulate(Robot.Articulation.PREP_FOR_HANG);
                }
            }
        }

        if (stickyGamepad1.x && CenterStage_6832.totalRunTime > 110/*field.getZone(robot.driveTrain.pose) != null && field.getZone(robot.driveTrain.pose).name.equals("AUDIENCE")*/) {
            robot.intake.setIngestPixelHeight(robot.intake.getIngestPixelHeight()-1);
        }
        else if(stickyGamepad1.x) {
            robot.articulate(Robot.Articulation.LAUNCH_DRONE);
        }

        if(stickyGamepad1.dpad_up) {
            robot.intake.articulate(Intake.Articulation.SWALLOW);
        }

        if (stickyGamepad1.dpad_down) {
            robot.outtake.setTargetAngle(Outtake.FLIPPER_TRAVEL_ANGLE);
        }
        // ------------------------------------------------------------------

        //GAMEPAD 2 CONTROLS
        // ------------------------------------------------------------------
        if(stickyGamepad2.guide) {
            robot.driveTrain.pose = new Pose2d(robot.driveTrain.pose.position, robot.driveTrain.imuAngle);
        }

        if(stickyGamepad2.b) {
            fieldOrientedDrive  = !fieldOrientedDrive;
        }

        if(stickyGamepad2.y) {
            if(robot.skyhook.articulation.equals(Skyhook.Articulation.PREP_FOR_HANG)) {
                robot.articulate(Robot.Articulation.HANG);
            }
            else {
                robot.articulate(Robot.Articulation.PREP_FOR_HANG);
            }
        }

        if(stickyGamepad2.a) {
            robot.skyhook.articulate(Skyhook.Articulation.INIT);
        }

        if(stickyGamepad2.x) {
            robot.articulate(Robot.Articulation.LAUNCH_DRONE);
        }
        // ------------------------------------------------------------------

        //mu name is jimmy and i am a pickle and i am a potato and i need to sleep with my truffle oil to feel happiness

    }

    public void fieldOrientedDrive() {
        if(Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                Math.abs(gamepad1.right_stick_x ) > DEADZONE)
        {
            robot.driveTrain.setHumanIsDriving(true);
            robot.driveTrain.fieldOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, alliance.getMod());
        }
        else if (robot.driveTrain.isHumanIsDriving()) robot.driveTrain.drive(0, 0, 0);
    }

    public void robotOrientedDrive() {
        if(Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                Math.abs(gamepad1.right_stick_x ) > DEADZONE) {
            robot.driveTrain.setHumanIsDriving(true);
            robot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }
        else if (robot.driveTrain.isHumanIsDriving()) robot.driveTrain.drive(0, 0, 0);
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
            visionProviderFinalized = false;
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
            visionProviderFinalized = false;
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
