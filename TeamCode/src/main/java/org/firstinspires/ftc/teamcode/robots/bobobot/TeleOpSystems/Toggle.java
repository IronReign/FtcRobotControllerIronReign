package org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import static org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning.BoboRunnerOp.runnerBot;

import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

public class Toggle {

    Gamepad gamepad1;
    private StickyGamepad stickyGamepad1;

    public Toggle(Gamepad gamepad1){
        this.gamepad1 = gamepad1;
        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    public void gamepadUpdate(){
        stickyGamepad1.update();
    }

    public void toggleSpeedMode(){
        if(stickyGamepad1.x){
            runnerBot.driveTrain.modeToggle();
        }
    }

    public void motorDebugTest(){
        if(stickyGamepad1.a){
            runnerBot.driveTrain.runTest();
        }
    }
}
