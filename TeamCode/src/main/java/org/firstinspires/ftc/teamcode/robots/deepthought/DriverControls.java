package org.firstinspires.ftc.teamcode.robots.deepthought;


import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.startingPosition;


import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident.shoulderSpeed;
import static org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.Sampler.SLIDE_ADJUST_SPEED;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.Arm;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.Sampler;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.SpeciMiner;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;

//hi this is fern
public class DriverControls {
    //CONSTANTS
    public static boolean fieldOrientedDrive = true;
    public static double DEADZONE = 0.1;
    public static double driveDampener = .5;

    public static void setDampenDrive(boolean dampenDrive) {
        DriverControls.dampenDrive = dampenDrive;
    }

    public static boolean dampenDrive = false;
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
        dampenDrive = true;
        updateStickyGamepads();
        handleStateSwitch();
        handlePregameControls();
    }

    public void updateStickyGamepads() {
        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    public void relocalizationTestMethods() {
        robot.driveTrain.DirectDriveMecanums(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.right_bumper) {
            Robot.panTargetPosition = Robot.PAN_FORWARD;
            robot.alignOnSample();
        }

        if (gamepad1.left_bumper) {
            robot.limelight.pipelineSwitch(2);
            Robot.panTargetPosition = Robot.PAN_BASKET_APRILTAG;
            robot.aprilTagRelocalization();
        }

        if (gamepad1.dpad_up) {
            robot.limelight.pipelineSwitch(4);
            Robot.panTargetPosition = Robot.PAN_SPECIMINER_APRILTAG;
            robot.aprilTagRelocalization();
        }
        if(gamepad1.dpad_up) {
            robot.forwardRelocalize();
        }
    }

    public void manualDiagnosticMethods() {
        robotOrientedDrive();
        //adjust shoulder
        if (gamepad1.right_bumper) {
            robot.trident.adjustShoulder(-shoulderSpeed);
        }
        if (gamepad1.left_bumper) {
            robot.trident.adjustShoulder(shoulderSpeed);        }

//
        if (gamepad1.left_trigger > .2) {
            robot.trident.sampler.articulate(Sampler.Articulation.MANUAL);
            robot.trident.speciMiner.articulate(SpeciMiner.Articulation.MANUAL);
            Trident.enforceSlideLimits = false;
            robot.trident.sampler.adjustSlide(-SLIDE_ADJUST_SPEED);
            robot.trident.speciMiner.adjustSlide(-SLIDE_ADJUST_SPEED);
        }
        if (gamepad1.right_trigger > .2) {
            robot.trident.sampler.articulate(Sampler.Articulation.MANUAL);
            robot.trident.speciMiner.articulate(SpeciMiner.Articulation.MANUAL);
            Trident.enforceSlideLimits = false;
            robot.trident.sampler.adjustSlide(SLIDE_ADJUST_SPEED);
            robot.trident.speciMiner.adjustSlide(SLIDE_ADJUST_SPEED);
        }
//
        if (stickyGamepad1.a) {
            robot.resetStates();
            robot.trident.getActiveArm().currentSample = Arm.Sample.NO_SAMPLE;

            dampenDrive = true;

            if (robot.trident.getActiveArm() instanceof Sampler) {
                if (robot.trident.sampler.articulation == Sampler.Articulation.INTAKE_PREP) {
                    robot.resetStates();
                    robot.articulate(Robot.Articulation.SAMPLER_INTAKE);
                } else {
                    robot.articulate(Robot.Articulation.SAMPLER_PREP);
                }
            } else {
                robot.articulate(Robot.Articulation.SPECIMINER_WALLTAKE);
            }
        }
//
        if (stickyGamepad1.b) {
//            robot.trident.getActiveArm().currentSample = Arm.Sample.NO_SAMPLE;

            dampenDrive = true;

            if (robot.trident.getActiveArm() instanceof Sampler) {
                robot.resetStates();
                robot.articulate(Robot.Articulation.SAMPLER_OUTTAKE);
            } else {
                if (robot.trident.speciMiner.articulation == SpeciMiner.Articulation.OUTTAKE || robot.articulation == Robot.Articulation.SPECIMINER_OUTTAKE) {
                    robot.trident.speciMiner.incrementOuttake();
                } else {
                    robot.resetStates();
                    robot.articulate(Robot.Articulation.SPECIMINER_OUTTAKE);
                }
            }
        }

        if (stickyGamepad1.guide) {
            robot.driveTrain.setPose(new Pose2d(robot.driveTrain.getPose().position, Math.toRadians(alliance.isRed() ? 90 : -90)));
        }

        double power;
        if (stickyGamepad1.x) {
            if (robot.trident.sampler.articulation == Sampler.Articulation.INTAKE) {
                robot.trident.sampler.currentSample = Arm.Sample.NEUTRAL;
            } else {
                if (robot.trident.getActiveArm() instanceof Sampler) {
                    robot.trident.getActiveArm().servoPower = robot.trident.getActiveArm().servoPower == .8 ? 0 : .8;
                } else {
                    power = robot.trident.getActiveArm().servoPower;
                    if (power == 1) {
                        robot.trident.getActiveArm().servoPower = 0;
                    }
                    if (power == 0) {
                        robot.trident.getActiveArm().servoPower = -1;
                    }
                    if (power == -1) {
                        robot.trident.getActiveArm().servoPower = 1;
                    }
                }

            }
        }

        if (stickyGamepad1.y) {
            dampenDrive = false;
            robot.resetStates();
            robot.articulate(Robot.Articulation.TRAVEL);
        }

        if (stickyGamepad1.start) {
            Sampler.colorSensorEnabled = !Sampler.colorSensorEnabled;
            debugTelemetryEnabled = true;
        }

        if (gamepad1.dpad_down) {
            robot.trident.sampler.adjustElbow(Arm.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad1.dpad_up) {
            robot.trident.sampler.adjustElbow(-Arm.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad1.dpad_left) {
            robot.trident.speciMiner.adjustElbow(Arm.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad1.dpad_right) {
            robot.trident.speciMiner.adjustElbow(-Arm.ELBOW_ADJUST_ANGLE);
        }

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
        if (fieldOrientedDrive)
            fieldOrientedDrive();
        else
            robotOrientedDrive();
        // adjust shoulder
        if (gamepad1.right_bumper) {
            robot.trident.adjustShoulder(-shoulderSpeed);
        }
        if (gamepad1.left_bumper) {
            robot.trident.adjustShoulder(shoulderSpeed);
        }

        if (gamepad1.left_trigger > .2) {
            Trident.enforceSlideLimits = false;
            robot.trident.getActiveArm().adjustSlide(-SLIDE_ADJUST_SPEED);
        }
        if (gamepad1.right_trigger > .2) {
            Trident.enforceSlideLimits = false;
            robot.trident.getActiveArm().adjustSlide(SLIDE_ADJUST_SPEED);
        }
//
        if (stickyGamepad1.a) {
            robot.resetStates();
            robot.trident.getActiveArm().currentSample = Arm.Sample.NO_SAMPLE;

            dampenDrive = true;

            if (robot.trident.getActiveArm() instanceof Sampler) {
                if (robot.trident.sampler.articulation == Sampler.Articulation.INTAKE_PREP) {
                    robot.resetStates();
                    robot.articulate(Robot.Articulation.SAMPLER_INTAKE);
                } else {
                    robot.articulate(Robot.Articulation.SAMPLER_PREP);
                }
            } else {
                robot.articulate(Robot.Articulation.SPECIMINER_WALLTAKE);
            }
        }
//
        if (stickyGamepad1.b) {
//            robot.trident.getActiveArm().currentSample = Arm.Sample.NO_SAMPLE;

            dampenDrive = true;

            if (robot.trident.getActiveArm() instanceof Sampler) {
                robot.resetStates();
                robot.articulate(Robot.Articulation.SAMPLER_OUTTAKE);
            } else {
                if (robot.trident.speciMiner.articulation == SpeciMiner.Articulation.OUTTAKE || robot.articulation == Robot.Articulation.SPECIMINER_OUTTAKE) {
                    robot.trident.speciMiner.incrementOuttake();
                } else {
                    robot.resetStates();
                    robot.articulate(Robot.Articulation.SPECIMINER_OUTTAKE);
                }
            }
        }

        if (stickyGamepad1.guide) {
            robot.aprilTagRelocalization();
        }

        double power;
        if (stickyGamepad1.x) {
            if (robot.trident.sampler.articulation == Sampler.Articulation.INTAKE) {
                robot.trident.sampler.currentSample = Arm.Sample.NEUTRAL;
            } else {
                if (robot.trident.getActiveArm() instanceof Sampler) {
                    robot.trident.getActiveArm().servoPower = robot.trident.getActiveArm().servoPower == .8 ? 0 : .8;
                } else {
                    power = robot.trident.getActiveArm().servoPower;
                    if (power == 1) {
                        robot.trident.getActiveArm().servoPower = 0;
                    }
                    if (power == 0) {
                        robot.trident.getActiveArm().servoPower = -1;
                    }
                    if (power == -1) {
                        robot.trident.getActiveArm().servoPower = 1;
                    }
                }

            }
        }

        if (stickyGamepad1.y) {
            dampenDrive = false;
            robot.resetStates();
            robot.articulate(Robot.Articulation.TRAVEL);
        }

        if (stickyGamepad1.start) {
            Sampler.colorSensorEnabled = !Sampler.colorSensorEnabled;
            debugTelemetryEnabled = true;
        }

        if (gamepad1.dpad_down) {
            robot.trident.getActiveArm().adjustElbow(Arm.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad1.dpad_up) {
            robot.trident.getActiveArm().adjustElbow(-Arm.ELBOW_ADJUST_ANGLE);
        }
        if (stickyGamepad1.dpad_left) {
            robot.trident.setActiveArm(robot.trident.sampler);
        }
        if (stickyGamepad1.dpad_right) {
            robot.trident.setActiveArm(robot.trident.speciMiner);

        }
        // ------------------------------------------------------------------
        //GAMEPAD 2 CONTROLS
        // ------------------------------------------------------------------
        if (stickyGamepad2.b) {
            fieldOrientedDrive = !fieldOrientedDrive;
        }
        if (gamepad2.left_trigger > .1) {
            robot.trident.sampler.adjustElbow(-Arm.ELBOW_ADJUST_ANGLE);
        }
        if (gamepad2.right_trigger > .1) {
            robot.trident.sampler.adjustElbow(Arm.ELBOW_ADJUST_ANGLE);
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
            if (dampenDrive)
                robot.driveTrain.fieldOrientedDrive(gamepad1.left_stick_x * driveDampener, gamepad1.left_stick_y * driveDampener, gamepad1.right_stick_x * driveDampener, alliance.isRed());
            else
                robot.driveTrain.fieldOrientedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, alliance.isRed());
        } else robot.driveTrain.drive(0, 0, 0);
    }

    public void robotOrientedDrive() {
        if (DriveTrain.roadRunnerDrive) {
            if (Math.abs(gamepad1.left_stick_x) > DEADZONE ||
                    Math.abs(gamepad1.left_stick_y) > DEADZONE ||
                    Math.abs(gamepad1.right_stick_x) > DEADZONE) {
                if (dampenDrive)
                    robot.driveTrain.drive(gamepad1.left_stick_x * driveDampener, gamepad1.left_stick_y * driveDampener, -gamepad1.right_stick_x * driveDampener);
                else
                    robot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            } else
                robot.driveTrain.drive(0, 0, 0);
        } else
            robot.driveTrain.DirectDriveMecanums(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }


    public void handleStateSwitch() {
        if (stickyGamepad1.a) {
            IntoTheDeep_6832.gameStateIndex = 0;
        }

        if (stickyGamepad1.y) {
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

        if (stickyGamepad1.guide) {
            robot.trident.calibrateIndex = 0;
            robot.trident.articulate(Trident.Articulation.CALIBRATE);
        }
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
//            robot.visionProviderBack.setRedAlliance(false);
        }
        if (stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Constants.Alliance.RED;
            visionProviderFinalized = false;
            startingPosition = startingPosition.isRed() == true ?
                    startingPosition :
                    startingPosition == Constants.Position.START_LEFT_BLUE ?
                            Constants.Position.START_LEFT_RED : Constants.Position.START_RIGHT_RED;
//            robot.visionProviderBack.setRedAlliance(true);
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
