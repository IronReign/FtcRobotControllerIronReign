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
}
