package org.firstinspires.ftc.teamcode.robots.bobobot.Utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.robots.bobobot.BoboDebugOp.debugbot;
import static org.firstinspires.ftc.teamcode.robots.bobobot.BoboRunnerOp.runnerBot;
import static org.firstinspires.ftc.teamcode.robots.bobobot.MotorDebug.motorPower;
import org.firstinspires.ftc.teamcode.robots.bobobot.BoboRunnerOp;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

public class Toggle {

    Gamepad gamepad1;
    Gamepad gamepad2;
    private StickyGamepad stickyGamepad1;
    private StickyGamepad stickyGamepad2;

    public Toggle(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
    }

    public void gamepadUpdate(){
        stickyGamepad1.update(); stickyGamepad2.update();
    }

    public void toggleSpeedMode(){
        if(stickyGamepad1.x){
            runnerBot.driveTrain.modeToggle();
        }
    }

    public void runTest(){
        if(stickyGamepad2.a){
            debugbot.motorDebug.motorDebugTest();
        }
    }

    public void armUp(){
        if(gamepad1.a){
            runnerBot.intake.clawArmLift();
        }
    }

    public void armDown(){
        if(gamepad1.b){
            runnerBot.intake.clawArmLower();
        }
    }

    public void intake(){
        if(gamepad1.b){
            runnerBot.intake.clawArmLift();
        }
        if(gamepad1.a){
            runnerBot.intake.clawArmLower();
        }

        if(gamepad1.right_bumper){
            runnerBot.intake.openClaw();
        }

        if(gamepad1.left_bumper){
            runnerBot.intake.closeClaw();
        }

        if(gamepad1.dpad_up){
            runnerBot.intake.armWristOut();
        }

        if (gamepad1.dpad_down){
            runnerBot.intake.armWristIn();
        }
    }

    public void drone(){
        if(gamepad1.y){
            runnerBot.drone.droneRelease();
        }
    }
}
