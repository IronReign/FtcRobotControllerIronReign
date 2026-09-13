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

    public DriverControls( Gamepad gamepad1, Robot robot)
    {
        this.gamepad1 = gamepad1;
        this.robot = robot;
    }

    public void update(){
        handleJoystickDrive();
        handleIntake();
        //handleFlywheel();
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
