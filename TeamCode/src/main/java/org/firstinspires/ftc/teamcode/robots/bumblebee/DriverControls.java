package org.firstinspires.ftc.teamcode.robots.bumblebee;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Intake;

public class DriverControls {


    Robot robot;
    Gamepad gamepad1;

    private boolean intake = false;

    public DriverControls(HardwareMap hardwareMap, Gamepad gamepad){
        robot = new Robot(hardwareMap);
        gamepad1 = gamepad;
    }

    public void update(){
        handleDrivetrain();
        handleIntake();
    }

    public void handleDrivetrain(){

    }

    public void handleIntake(){
        if(gamepad1.right_bumper || gamepad1.left_bumper) intake = !intake;

        if(intake && gamepad1.right_bumper) {
            robot.intake.setBehavior(Intake.IntakeState.INTAKING);
        } else if( intake && gamepad1.left_bumper){
            robot.intake.setBehavior(Intake.IntakeState.EJECTING);
        }else if(!intake){
            robot.intake.setBehavior(Intake.IntakeState.OFF);
        }

    }

}
