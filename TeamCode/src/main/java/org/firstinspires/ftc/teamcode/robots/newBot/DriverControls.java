package org.firstinspires.ftc.teamcode.robots.newBot;

public class DriverControls {


    

    public void joystickDrive() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        
        robot.driveTrain.drive(
                forward,
                strafe,
                turn
        );
    }
}
