package org.firstinspires.ftc.teamcode.robots.newBot;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;


@Config(value = "newBot_DriverControls")
public class DriverControls {

    private final Gamepad gamepad1;
    private final Robot robot;

    public DriverControls( Gamepad gamepad1, Robot robot)
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
        
        robot.driveTrain.drive( forward, strafe, turn );
    }
}
