package org.firstinspires.ftc.teamcode.robots.bobby;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveToAprilTagOmni;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

public class DriverControls {
    private boolean flywheelOn = false;

    public void update(Robot robot, Gamepad gamepad, StickyGamepad stickyGamepad, LaunchSequence launchSequence) {
        robot.handleDrive(gamepad);

        if (gamepad.y && !launchSequence.isRunning()) {
            flywheelOn = true;
            launchSequence.start(robot);
        }

        launchSequence.update(robot);

        if (!launchSequence.isRunning()) {
            handlePusher(robot, gamepad);
            handleIntake(robot, gamepad);
            handleFlywheel(robot, stickyGamepad);
        } else {
            flywheelOn = true;
        }
    }

    private void handlePusher(Robot robot, Gamepad gamepad) {
        if (gamepad.b) {
            robot.setPusherUp();
        } else {
            robot.setPusherDown();
        }
    }

    private void handleIntake(Robot robot, Gamepad gamepad) {
        if (gamepad.right_trigger > 0.0) {
            robot.setIntakePower(1.0);
        } else if (gamepad.left_bumper) {
            robot.setIntakePower(-1.0);
        } else {
            robot.setIntakePower(0.0);
        }
    }

    private void handleFlywheel(Robot robot, StickyGamepad stickyGamepad) {
        if (stickyGamepad.right_bumper) {
            flywheelOn = !flywheelOn;
        }

        if (flywheelOn) {
            robot.setFlywheelVelocity(100);
        } else {
            robot.setFlywheelVelocity(0.0);
        }
    }
}