package org.firstinspires.ftc.teamcode.robots.deepthought;


import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident.colorSensorEnabled;
import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident.craneSpeed;
import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident.slideSpeed;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;

import java.util.Arrays;

public class DriverControls {
    //CONSTANTS
    public static boolean fieldOrientedDrive = false;
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
        handleStateSwitch();
        handlePregameControls();
    }

    public void updateStickyGamepads() {
        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    public void manualDiagnosticMethods() {
        robotOrientedDrive();
//        if(gamepad1.right_bumper) {
//            robot.trident.craneTargetPosition -= 1 * craneSpeed;
//        }
//        if(gamepad1.left_bumper) {
//            robot.trident.craneTargetPosition += 1 * craneSpeed;
//        }
//
//        if(gamepad1.dpad_right) {
//            robot.trident.slideTargetPosition -= 1 * slideSpeed;
//        }
//        if(gamepad1.dpad_left) {
//            robot.trident.slideTargetPosition -= 1 * slideSpeed;
//        }
//
//        if(stickyGamepad1.a) {
//            Trident.intakeIndex = 0;
//            robot.articulate(Robot.Articulation.INTAKE);
//        }
//
//        if(stickyGamepad1.b) {
//            Trident.outtakeIndex = 0;
//            robot.articulate(Robot.Articulation.OUTTAKE);
//        }

        if(stickyGamepad1.x) {
            robot.trident.targetSamples.add(Trident.CurrentSample.RED);
            robot.trident.beaterPower = 1;
            Trident.colorSensorEnabled = true;
        }
        robot.trident.stopOnSample();

        if(stickyGamepad1.start) {
            Trident.colorSensorEnabled = !colorSensorEnabled;
            debugTelemetryEnabled = true;
        }



//        if (gamepad1.left_trigger > .1) {
//            robot.trident.adjustWrist(-Trident.WRIST_ADJUST_ANGLE);
//        }
//        if (gamepad1.right_trigger > .1) {
//            robot.trident.adjustWrist(Trident.WRIST_ADJUST_ANGLE);
//        }
//
//        if (gamepad1.dpad_down) {
//            robot.trident.adjustElbow(Trident.ELBOW_ADJUST_ANGLE);
//        }
//        if (gamepad1.dpad_up) {
//            robot.trident.adjustElbow(-Trident.ELBOW_ADJUST_ANGLE);
//        }

    }

    public boolean joysticksInactive() {
        return gamepad1.left_stick_x < DEADZONE && gamepad1.left_stick_y < DEADZONE
                && gamepad1.right_stick_x < DEADZONE && gamepad1.right_stick_y < DEADZONE
                && gamepad2.left_stick_x < DEADZONE && gamepad2.left_stick_y < DEADZONE
                && gamepad2.right_stick_x < DEADZONE && gamepad2.right_stick_x < DEADZONE;
    }

    public void rumble(int gamepad, int duration) {
        if (gamepad == 1)
            gamepad1.rumble(duration);
        else
            gamepad2.rumble(duration);
    }

    public void joystickDrive() {

        //GAMEPAD 1 CONTROLS
        // ------------------------------------------------------------------
        if(!fieldOrientedDrive)
            robotOrientedDrive();
        else
            fieldOrientedDrive();

        if(stickyGamepad1.a) {
            Trident.intakeIndex = 0;
            robot.articulate(Robot.Articulation.INTAKE);
        }
        if(stickyGamepad1.b) {
            Trident.outtakeIndex = 0;
            robot.articulate(Robot.Articulation.OUTTAKE);
        }
        if(stickyGamepad1.y) {
            Trident.tuckIndex = 0;
            robot.articulate(Robot.Articulation.TRAVEL);
        }

        if(stickyGamepad1.x) {
            robot.trident.beaterPower = robot.trident.beaterPower == 1? 0 : 1;
        }


        //adjust methods
        if(gamepad1.dpad_up) {
            robot.trident.slideTargetPosition += slideSpeed;
        }
        if(gamepad1.dpad_down) {
            robot.trident.slideTargetPosition -= slideSpeed;
        }
        if(gamepad1.right_bumper) {
            robot.trident.adjustElbow(Trident.ELBOW_ADJUST_ANGLE);
        }
        if(gamepad1.left_bumper) {
            robot.trident.adjustElbow(-Trident.ELBOW_ADJUST_ANGLE);
        }
        if(gamepad1.left_trigger > .2) {
            robot.trident.adjustWrist(-Trident.WRIST_ADJUST_ANGLE);
        }
        if(gamepad1.right_trigger > .2) {
            robot.trident.adjustWrist(Trident.WRIST_ADJUST_ANGLE);
        }
        //


        // ------------------------------------------------------------------
        //GAMEPAD 2 CONTROLS
        // ------------------------------------------------------------------
        if (stickyGamepad2.b) {
            fieldOrientedDrive = !fieldOrientedDrive;
        }
        if (gamepad2.left_trigger > .1) {
            robot.trident.adjustElbow(-robot.trident.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad2.right_trigger > .1) {
            robot.trident.adjustElbow(robot.trident.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad2.left_bumper) {
            robot.trident.adjustWrist(-robot.trident.WRIST_ADJUST_ANGLE);
        }
        if (gamepad2.right_bumper) {
            robot.trident.adjustWrist(robot.trident.WRIST_ADJUST_ANGLE);
        }
        // ------------------------------------------------------------------

    }

    public boolean shifted(Gamepad gamepad) {
        return gamepad.guide;
    }

    public void fieldOrientedDrive() {
        if (Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                Math.abs(gamepad1.right_stick_x) > DEADZONE) {
            robot.driveTrain.fieldOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, alliance.isRed());
        } else robot.driveTrain.drive(0, 0, 0);
    }

    public void robotOrientedDrive() {
        if (DriveTrain.roadRunnerDrive) {
            if (Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                    Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                    Math.abs(gamepad1.right_stick_x) > DEADZONE) {
                robot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }
            else
                robot.driveTrain.drive(0, 0, 0);
        }
        else
            robot.driveTrain.mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }


    public void handleStateSwitch() {
        if(stickyGamepad1.a) {
            IntoTheDeep_6832.gameStateIndex = 0;
        }

        if(stickyGamepad1.y){
            IntoTheDeep_6832.gameStateIndex = 1;
        }

             if (stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
                IntoTheDeep_6832.gameStateIndex -= 1;
            if (stickyGamepad1.right_bumper || stickyGamepad2.right_bumper)
                IntoTheDeep_6832.gameStateIndex += 1;
            if (IntoTheDeep_6832.gameStateIndex < 0)
                IntoTheDeep_6832.gameStateIndex = IntoTheDeep_6832.GameState.getNumGameStates() - 1;
            IntoTheDeep_6832.gameStateIndex %= IntoTheDeep_6832.GameState.getNumGameStates();
            IntoTheDeep_6832.gameState = IntoTheDeep_6832.GameState.getGameState(IntoTheDeep_6832.gameStateIndex);




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
        }
        if (stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Constants.Alliance.RED;
            visionProviderFinalized = false;
            startingPosition = startingPosition.isRed() == true ?
                    startingPosition :
                    startingPosition == Constants.Position.START_LEFT_BLUE ?
                            Constants.Position.START_LEFT_RED : Constants.Position.START_RIGHT_RED;
            robot.visionProviderBack.setRedAlliance(true);
        }

        if (stickyGamepad1.dpad_up) {
            robot.visionProviderFinalized = false;
        }

        if (stickyGamepad1.dpad_left || stickyGamepad2.dpad_left)
            startingPosition = alliance == Constants.Alliance.RED ? Constants.Position.START_LEFT_RED : Constants.Position.START_LEFT_BLUE;

        if (stickyGamepad1.dpad_right || stickyGamepad2.dpad_right)
            startingPosition = alliance == Constants.Alliance.RED ? Constants.Position.START_RIGHT_RED : Constants.Position.START_RIGHT_BLUE;
    }


}
