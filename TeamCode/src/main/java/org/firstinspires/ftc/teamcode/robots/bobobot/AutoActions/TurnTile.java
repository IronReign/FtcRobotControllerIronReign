package org.firstinspires.ftc.teamcode.robots.bobobot.AutoActions;

import org.firstinspires.ftc.teamcode.robots.bobobot.Assign;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;
import static org.firstinspires.ftc.teamcode.robots.bobobot.Auton.turnDone;
public class TurnTile extends Assign {
    private Autobot autobot;
    private double degrees;

    @Override
    public double getDelta() {
        return delta;
    }
    public TurnTile(Autobot autobot, double degrees){
        this.autobot = autobot;
        if(degrees < 0){
            this.degrees = degrees + 360;
        }
        else {
            this.degrees = degrees;
        }
        delta = autobot.drive.getMotorAvgPosition() + Math.abs((degrees/90)*TICKSPER90);
    }

    @Override
    public boolean run(){
        if(autobot.drive.getMotorAvgPosition() < delta){
            autobot.drive.mecanumAuto(0, 0, degrees/Math.abs(degrees)*MAXMOTORSPEED);

            return true;
        }
        else{
            turnDone = true;
            autobot.drive.resetMotors();
            return false;
        }
    }
}
