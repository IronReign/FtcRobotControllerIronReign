package org.firstinspires.ftc.teamcode.robots.bobobot.Utilities;

import com.qualcomm.robotcore.hardware.Gamepad;
import static org.firstinspires.ftc.teamcode.robots.bobobot.BoboRunnerOp.runnerBot;
import static org.firstinspires.ftc.teamcode.robots.bobobot.MotorDebug.motorPower;
import org.firstinspires.ftc.teamcode.robots.bobobot.BoboRunnerOp;
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

    public void motorDebugTest() {
        if (stickyGamepad1.dpad_left) {
        }
    }
    public void toggleDebugMode () {
        if (stickyGamepad1.dpad_right){
            runnerBot.driveTrain.debugSwitch();
            }
        }


}
