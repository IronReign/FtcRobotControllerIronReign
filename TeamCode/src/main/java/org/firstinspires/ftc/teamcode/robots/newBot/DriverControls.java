package org.firstinspires.ftc.teamcode.robots.newBot;

@Config(value = "newBot_DriverControls")
public class DriverControls {

    private final Gamepad gamepad1;
    private final NewBot robot;

    public DriverControls( Gamepad gamepad1, NewBot robot)
    {
        this.gamepad1 = gamepad1;
        this.robot = robot;
    }

    public void update(){
        handleJoystickDrive();
    }

    public void handleJoystickDrive() {

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
