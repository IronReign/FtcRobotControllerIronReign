package org.firstinspires.ftc.teamcode.robots.ri2d2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;


@Config(value = "newBot_DriverControls")
public class DriverControls {

    private final Gamepad gamepad1;
    private final Robot robot;
    private boolean previousA = false;

    public static double intakePower = -0.7;
    private boolean intake = false;
    private boolean previousRightBumper = false;

    private boolean previousY = false;
    private boolean previousX = false;
    private boolean previousB = false;
    private boolean previousBack = false;

    public DriverControls( Gamepad gamepad1, Robot robot)
    {
        this.gamepad1 = gamepad1;
        this.robot = robot;
    }

    public void update(){
        handleJoystickDrive();
        handleIntake();
        handleCatapult();
        //handleFlywheel();
    }

    private void handleCatapult() {
        // Y = arm: winds down, latches, slackens, ends at READY ready to fire
        if (gamepad1.y && !previousY) {
            robot.catapult.retract();
        }
        previousY = gamepad1.y;

        // X = launch. Slackens the string first if it isn't already slack.
        if (gamepad1.x && !previousX) {
            robot.catapult.launch();
        }
        previousX = gamepad1.x;

        // B = abort whatever the catapult is doing
        if (gamepad1.b && !previousB) {
            robot.catapult.abort();
        }
        previousB = gamepad1.b;

        // BACK = zero the spool encoder at the current position (setup only)
        if (gamepad1.back && !previousBack) {
            robot.catapult.zeroEncoder();
        }
        previousBack = gamepad1.back;

        // Dpad jogs the spool by hand so we can rig the string and find positions
        if (gamepad1.dpad_up) {
            robot.catapult.jog(1);
        } else if (gamepad1.dpad_down) {
            robot.catapult.jog(-1);
        } else {
            robot.catapult.jog(0);
        }
    }

    private void handleIntake() {
        // Toggle once per press, even when the right bumper is held.
        if (gamepad1.right_bumper && !previousRightBumper) {
            intake = !intake;
        }
        previousRightBumper = gamepad1.right_bumper;
        robot.intake.setPower(intake ? intakePower : 0);
    }

//    private void handleFlywheel() {
//        if (gamepad1.a && !previousA) {
//            robot.flywheel.toggle();
//        }
//        previousA = gamepad1.a;
//    }

    public void handleJoystickDrive() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        
        robot.driveTrain.drive( forward, strafe, turn );
    }
}
