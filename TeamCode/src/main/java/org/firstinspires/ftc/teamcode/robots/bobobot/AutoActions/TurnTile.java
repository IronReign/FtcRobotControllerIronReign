package org.firstinspires.ftc.teamcode.robots.bobobot.AutoActions;

import org.firstinspires.ftc.teamcode.robots.bobobot.Assign;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;
import static org.firstinspires.ftc.teamcode.robots.bobobot.Auton.turnDone;
public class TurnTile extends Assign {
    private Autobot autobot;
    private double degrees;
    private double targetHeading;
    private int timer = (int) (System.currentTimeMillis() + 250);

    @Override
    public double getDelta() {
        return delta;
    }
    public TurnTile(Autobot autobot, double degrees){
        this.autobot = autobot;
        this.degrees = degrees;
        targetHeading = degrees;
        delta = autobot.drive.getMotorAvgPosition() + Math.abs((degrees/180)*TICKSPER90);
    }

    @Override
    public boolean run(){
        if(autobot.drive.getHeading() < targetHeading && autobot.drive.getMotorAvgPosition() < delta /*|| System.currentTimeMillis() < timer*/){
            autobot.drive.mecanumAuto(0, 0, Math.signum(degrees)*MAXMOTORSPEED);
            return true;
        }
        else{
            turnDone = true;
            autobot.drive.resetMotors();
            return false;
        }
    }
}
