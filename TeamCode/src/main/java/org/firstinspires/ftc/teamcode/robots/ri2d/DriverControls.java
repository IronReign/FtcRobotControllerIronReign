package org.firstinspires.ftc.teamcode.robots.ri2d;

import static org.firstinspires.ftc.teamcode.robots.ri2d.ri2dOpMode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

public class DriverControls {

    public static double DEADZONE = 0.1;
    Gamepad gamepad1;
    private StickyGamepad stickyGamepad1;

    public DriverControls(Gamepad pad){
        gamepad1 = new Gamepad();
        stickyGamepad1 = new StickyGamepad(gamepad1);

    }

    public void init_loop(){

    }

    public void updatestickygamepad(){

    }

    public void joystickDrive(){
//        if(Math.abs(gamepad1.left_stick_x) > DEADZONE ||
//                Math.abs(gamepad1.left_stick_y) > DEADZONE ||
//                Math.abs(gamepad1.right_stick_x ) > DEADZONE)
//        {
        robot.swerve.simplySwerve(gamepad1.left_stick_x, gamepad1.left_stick_y);
    }
}
