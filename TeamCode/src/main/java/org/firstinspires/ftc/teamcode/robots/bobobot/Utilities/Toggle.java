package org.firstinspires.ftc.teamcode.robots.bobobot.Utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.robots.bobobot.BoboDebugOp.debugbot;
import static org.firstinspires.ftc.teamcode.robots.bobobot.BoboDebugOp.turn0;
import static org.firstinspires.ftc.teamcode.robots.bobobot.BoboDebugOp.turn180;
import static org.firstinspires.ftc.teamcode.robots.bobobot.BoboDebugOp.turn90;

import org.firstinspires.ftc.teamcode.robots.bobobot.BoboRunnerOp;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

public class Toggle {

    Gamepad gamepad1;
    Gamepad gamepad2;
    RunnerBot runnerBot;
    private StickyGamepad stickyGamepad1;
    private StickyGamepad stickyGamepad2;

    public Toggle(Gamepad gamepad1, Gamepad gamepad2, RunnerBot robot){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        this.runnerBot = robot;
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
            //debugbot.motorDebug.motorDebugTest();
        }
    }

    public void turnTest1(){
        if(stickyGamepad1.y){
            turn90 = !turn90;
        }
    }

    public void turnTest2(){
        if(stickyGamepad1.x){
            turn180 = !turn180;
        }
    }

    public void turnTest3(){
        if(stickyGamepad1.b){
            turn0 = !turn0;
        }
    }

    public void intake(){
        if(gamepad1.b){
            runnerBot.intake.clawArmLift();
        }
        if(gamepad1.a){
            runnerBot.intake.clawArmLower();
        }
        if(gamepad1.dpad_right){
            runnerBot.intake.reset();
        }
        if(gamepad1.right_bumper){
            runnerBot.intake.openClaw();
        }

        if(gamepad1.left_bumper){
            runnerBot.intake.closeClaw();
        }

        if(gamepad1.dpad_down){
            runnerBot.intake.armWristOut();
        }

        if (gamepad1.dpad_up){
            BoboRunnerOp.runnerBot.intake.armWristIn();
        }

        if (gamepad1.dpad_left){
            BoboRunnerOp.runnerBot.intake.armScoreLift();
        }

        if(gamepad1.dpad_right){
            BoboRunnerOp.runnerBot.intake.armScoreLower();
        }

    }


    public void autonSetup(){
        if(stickyGamepad1.b){
            runnerBot.driveTrain.setPose(Constants.Position.START_LEFT_RED);
            DriveTrain.gamePosition = Constants.Position.START_LEFT_RED;
            DriveTrain.alliance = Constants.Alliance.RED;
            //runnerBot.visionProvider.setRedAlliance(true);
        }

        if(stickyGamepad1.y){
            runnerBot.driveTrain.setPose(Constants.Position.START_RIGHT_RED);
            DriveTrain.gamePosition = Constants.Position.START_RIGHT_RED;
            DriveTrain.alliance = Constants.Alliance.RED;
            //runnerBot.visionProvider.setRedAlliance(true);
        }

        if(stickyGamepad1.x){
            runnerBot.driveTrain.setPose(Constants.Position.START_RIGHT_BLUE);
            DriveTrain.gamePosition = Constants.Position.START_RIGHT_BLUE;
            DriveTrain.alliance = Constants.Alliance.BLUE;
            //runnerBot.visionProvider.setRedAlliance(false);

        }

        if(stickyGamepad1.a){
            runnerBot.driveTrain.setPose(Constants.Position.START_LEFT_BLUE);
            DriveTrain.gamePosition = Constants.Position.START_LEFT_BLUE;
            DriveTrain.alliance = Constants.Alliance.BLUE;
            //runnerBot.visionProvider.setRedAlliance(false);

        }

        if(stickyGamepad1.right_bumper){
            runnerBot.driveTrain.changeIndex();
        }
    }

    public void drone(){
        if(gamepad1.y){
            BoboRunnerOp.runnerBot.drone.droneRelease();
        }
    }

    int stateIndex = 0;
    public void switchState(boolean state){
        switch(stateIndex){
            case 0:
                state = true;
                stateIndex++;
                break;
            case 1:
                state = false;
                stateIndex = 0;
                break;
        }
    }
}
