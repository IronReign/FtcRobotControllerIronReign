package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.active;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.autoEndgameOn;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.autoNavOn;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.field;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident.SLIDE_ADJUST_SPEED;
import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot.visionOn;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Sensors;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Skyhook;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

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
        handleStateSwitch();
        handlePregameControls();
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

        if(gamepad1.right_trigger > DEADZONE)
            Skyhook.SKYHOOK_SAFE_TICKS += 10;
        if(gamepad1.left_trigger > DEADZONE)
            Skyhook.SKYHOOK_SAFE_TICKS -= 10;
        if(stickyGamepad1.y){
            Skyhook.droneLoaded = !Skyhook.droneLoaded;
        }
        if(stickyGamepad1.b){
            Sensors.skyhookIMUEnabled = !Sensors.skyhookIMUEnabled;
        }

        if(stickyGamepad1.left_stick_button) {
            visionOn = !visionOn;
        }

        fieldOrientedDrive();

    }

    public boolean joysticksInactive() {
        return gamepad1.left_stick_x < DEADZONE && gamepad1.left_stick_y < DEADZONE
                && gamepad1.right_stick_x < DEADZONE && gamepad1.right_stick_y < DEADZONE
                && gamepad2.left_stick_x < DEADZONE && gamepad2.left_stick_y < DEADZONE
                && gamepad2.right_stick_x < DEADZONE && gamepad2.right_stick_x < DEADZONE;
    }

    public void rumble(int gamepad, int duration){
        if(gamepad == 1)
            gamepad1.rumble(duration);
        else
            gamepad2.rumble(duration);
    }

    public void joystickDrive() {

        //GAMEPAD 1 CONTROLS
        // ------------------------------------------------------------------
        if (shifted(gamepad1) && gamepad1.left_trigger > .1) {
            robot.outtake.scoreX -= robot.outtake.IK_ADJUST_INCHES;
            if(robot.outtake.scoreX < 4)
                robot.outtake.scoreX = 4;
            robot.outtake.IKadjust(robot.outtake.scoreX, robot.outtake.scoreZ);
        }
        else if (gamepad1.left_trigger > .1) {
            if(robot.outtake.elevator.getCurrentAngle() > 0 && robot.outtake.scoreZ == 16) {
                robot.outtake.adjustElevator(-Outtake.ELEVATOR_ADJUST_ANGLE);
            }
            else {
                robot.outtake.scoreZ -= robot.outtake.IK_ADJUST_INCHES;
            }
            if(robot.outtake.scoreZ < 16)
                robot.outtake.scoreZ = 16;
            robot.outtake.elevatorIKAngle = robot.outtake.elevator.getTargetAngle();
            robot.outtake.IKadjust(robot.outtake.scoreX, robot.outtake.scoreZ);
        }
        if (shifted(gamepad1) && gamepad1.right_trigger > .1) {
            robot.outtake.scoreX += robot.outtake.IK_ADJUST_INCHES;
            if(robot.outtake.scoreX > 15)
                robot.outtake.scoreX = 15;
            robot.outtake.IKadjust(robot.outtake.scoreX, robot.outtake.scoreZ);
        }
        else if (gamepad1.right_trigger > .1) {
            robot.outtake.scoreZ += robot.outtake.IK_ADJUST_INCHES;
            if(robot.outtake.scoreZ > 21) {
                robot.outtake.scoreZ = 21;
                robot.outtake.adjustElevator(Outtake.ELEVATOR_ADJUST_ANGLE);
            }
            robot.outtake.elevatorIKAngle = robot.outtake.elevator.getTargetAngle();
            robot.outtake.IKadjust(robot.outtake.scoreX, robot.outtake.scoreZ);
        }

        if(shifted(gamepad1) && stickyGamepad1.a) {
            robot.skyhook.articulate(Skyhook.Articulation.TRAVEL);
        }
        else if (stickyGamepad1.a) {
            if(robot.articulation == Robot.Articulation.TRAVEL)
                robot.articulate(Robot.Articulation.INGEST);
        }

        if(shifted(gamepad1) && stickyGamepad1.b) {
            debugTelemetryEnabled = !debugTelemetryEnabled;
        }
        else if (stickyGamepad1.b) {
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
                robot.outtake.slide.setTargetPosition(robot.outtake.getSlideTargetPosition() -5 * SLIDE_ADJUST_SPEED);
        }
        if (gamepad1.right_bumper) {
            if (robot.intake.isEating())
                robot.intake.pixelSensorRight();
            else
                robot.outtake.slide.setTargetPosition(robot.outtake.getSlideTargetPosition() +5 * SLIDE_ADJUST_SPEED);
        }
        
        if(stickyGamepad1.y) {
            if(robot.skyhook.articulation.equals(Skyhook.Articulation.PREP_FOR_HANG)) {
                robot.articulate(Robot.Articulation.HANG);
            }
            else{
                if(shifted(gamepad1)/*field.getZone(robot.driveTrain.pose) != null && field.getZone(robot.driveTrain.pose).name.equals("AUDIENCE")*/) {
                    robot.articulate(Robot.Articulation.PREP_FOR_HANG);
                }
                else {
                    if(robot.intake.getIngestPixelHeight() != 4)
                        robot.intake.setIngestPixelHeight(4);
                    else if(robot.intake.getIngestPixelHeight() == 4)
                        robot.intake.setIngestPixelHeight(0);
                }
            }
        }

        if (stickyGamepad1.x && shifted(gamepad1)/*field.getZone(robot.driveTrain.pose) != null && field.getZone(robot.driveTrain.pose).name.equals("AUDIENCE")*/) {
            robot.articulate(Robot.Articulation.LAUNCH_DRONE);
        }
        else if(stickyGamepad1.x) {
            robot.intake.setIngestPixelHeight(robot.intake.getIngestPixelHeight()-1);
        }

        if(shifted(gamepad1) && gamepad1.dpad_up){
            robot.backdropRelocalize();
        }
        else if(stickyGamepad1.dpad_up) {
            robot.intake.articulate(Intake.Articulation.SETTLE);
        }

        if(stickyGamepad1.dpad_left) {
            if(CenterStage_6832.autoNav.setPreferredRoute(CenterStage_6832.autoNav.preferredRoute-1) < 0) {
                CenterStage_6832.autoNav.setPreferredRoute(6);
            }
        }
        if(stickyGamepad1.dpad_right) {
            if(CenterStage_6832.autoNav.setPreferredRoute(CenterStage_6832.autoNav.preferredRoute+1) > 6) {
                CenterStage_6832.autoNav.setPreferredRoute(0);
            }
        }
        if (shifted(gamepad1) && stickyGamepad1.start) {
            autoEndgameOn = !autoEndgameOn;
        }
        else if(stickyGamepad1.start){
            autoNavOn = !autoNavOn;
        }

        if (shifted(gamepad1) && stickyGamepad1.dpad_down) {
            fieldOrientedDrive = !fieldOrientedDrive;
        }
        else if(stickyGamepad1.dpad_down) {
            robot.outtake.articulate(Outtake.Articulation.TRAVEL_FROM_BACKDROP);
        }

        // ------------------------------------------------------------------

        //GAMEPAD 2 CONTROLS
        // ------------------------------------------------------------------
        if(stickyGamepad2.start) {
            if(CenterStage_6832.AUTONAV_ENABLED) {
                CenterStage_6832.autoNavOn = !CenterStage_6832.autoNavOn;
            }
        }
        else if(shifted(gamepad2) && stickyGamepad2.start) {
            CenterStage_6832.autoEndgameOn = !CenterStage_6832.autoEndgameOn;
        }
        if(stickyGamepad2.dpad_left) {
            if(CenterStage_6832.autoNav.setPreferredRoute(CenterStage_6832.autoNav.preferredRoute-1) < 0) {
                CenterStage_6832.autoNav.setPreferredRoute(6);
            }
        }
        if(stickyGamepad2.dpad_right) {
            if(CenterStage_6832.autoNav.setPreferredRoute(CenterStage_6832.autoNav.preferredRoute+1) > 6) {
                CenterStage_6832.autoNav.setPreferredRoute(0);
            }
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
        if(stickyGamepad2.x) {
            robot.articulate(Robot.Articulation.LAUNCH_DRONE);
        }
        if (gamepad2.left_trigger > .1) {
            robot.outtake.adjustElbow(-robot.outtake.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad2.right_trigger > .1) {
            robot.outtake.adjustElbow(robot.outtake.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad2.left_bumper) {
            robot.outtake.adjustWrist(-robot.outtake.WRIST_ADJUST_ANGLE);
        }
        if (gamepad2.right_bumper) {
            robot.outtake.adjustWrist(robot.outtake.WRIST_ADJUST_ANGLE);
        }
        // ------------------------------------------------------------------

    }

    public boolean shifted(Gamepad gamepad) {
        return gamepad.guide;
    }

    public void fieldOrientedDrive() {
        if(Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                Math.abs(gamepad1.right_stick_x ) > DEADZONE)
        {
            robot.driveTrain.setHumanIsDriving(true);
            robot.driveTrain.fieldOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, alliance.isRed());
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
                    startingPosition = startingPosition.isRed() == false ?
                            startingPosition :
                            startingPosition == Constants.Position.START_LEFT_RED ?
                                    Constants.Position.START_LEFT_BLUE : Constants.Position.START_RIGHT_BLUE;
            robot.visionProviderBack.setRedAlliance(false);
//            robot.visionProviderBack.initializeVision(hardwareMap); // this is blocking

        }
        if (stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Constants.Alliance.RED;
            visionProviderFinalized = false;
            startingPosition = startingPosition.isRed() == true ?
                    startingPosition :
                    startingPosition == Constants.Position.START_LEFT_BLUE ?
                            Constants.Position.START_LEFT_RED : Constants.Position.START_RIGHT_RED;
                robot.visionProviderBack.setRedAlliance(true);
//            robot.visionProviderBack.initializeVision(hardwareMap); // this is blocking

        }
        if(stickyGamepad1.start){
            Constants.driverSide = !Constants.driverSide;
            Constants.Position.resetStartPose();
        }

        if(stickyGamepad1.left_stick_button) {
            Constants.runPixelStack = !Constants.runPixelStack;
        }

        if(stickyGamepad1.y) {
            if(gameState.isAutonomous()) {
                robot.driveTrain.setPose(startingPosition);
            }
        }
        if(stickyGamepad1.right_stick_button) {
            field.finalizeField();
            robot.driveTrain.setPose(new Pose2d(field.SCORE6.pose.position, field.SCORE6.pose.heading.log()));
            robot.driveTrain.buildTestPathToWing();
            robot.driveTrain.runTestPath = true;
        }
        if(stickyGamepad1.guide) {
            robot.initPositionIndex ++;
        }

        if(stickyGamepad1.dpad_up) {
            robot.visionProviderFinalized = !robot.visionProviderFinalized;
        }

        if(stickyGamepad1.dpad_down || stickyGamepad2.dpad_down){
            Skyhook.droneLoaded = !Skyhook.droneLoaded;
        }

        if (stickyGamepad1.dpad_left || stickyGamepad2.dpad_left)
            startingPosition = alliance == Constants.Alliance.RED ? Constants.Position.START_LEFT_RED : Constants.Position.START_LEFT_BLUE;

        if (stickyGamepad1.dpad_right || stickyGamepad2.dpad_right)
            startingPosition = alliance == Constants.Alliance.RED ? Constants.Position.START_RIGHT_RED : Constants.Position.START_RIGHT_BLUE;

        if (stickyGamepad1.a || stickyGamepad2.a) {
            debugTelemetryEnabled = !debugTelemetryEnabled;
        }

    }


}
