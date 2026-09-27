package org.firstinspires.ftc.teamcode.robots.bumblebee;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Intake;

public class DriverControls {
    private final Robot robot;
    private final Gamepad gamepad1;

    private boolean intake = false;
    private boolean bumperWasPressed = false;

    public DriverControls(Robot robot, Gamepad gamepad1){
        this.robot = robot;
        this.gamepad1 = gamepad1;
    }

    public void update(){
        handleDrivetrain();
        handleIntake();
    }

    public void handleDrivetrain(){
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        robot.driveTrain.drive(forward, strafe, turn);
    }

    public void handleIntake(){
        boolean bumperPressed = gamepad1.right_bumper || gamepad1.left_bumper;

        if( bumperPressed && !bumperWasPressed) intake = !intake;

        bumperWasPressed = bumperPressed;

        if(intake && gamepad1.right_bumper) {
            robot.intake.setBehavior(Intake.IntakeState.INTAKING);
        } else if( intake && gamepad1.left_bumper){
            robot.intake.setBehavior(Intake.IntakeState.EJECTING);
        }else if(!intake){
            robot.intake.setBehavior(Intake.IntakeState.OFF);
        }

    }

}
