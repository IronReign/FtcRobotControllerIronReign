package org.firstinspires.ftc.teamcode.robots.bobobot.AutoTiles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.Assign;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;

public class TurnTile extends Assign {
    private Autobot autobot;
    private double degrees;
    //private double delta;
    @Override
    public double getDelta() {
        return delta;
    }
    public TurnTile(Autobot autobot, double degrees){
        this.autobot = autobot;
        this.degrees = degrees;
        delta = autobot.drive.getMotorAvgPosition() + Math.abs((degrees/90)*TICKSPER90);
    }

    @Override
    public boolean run(){
        if(autobot.drive.getMotorAvgPosition() < delta){
            autobot.drive.mecanumAuto(0, 0, degrees/Math.abs(degrees)*MAXMOTORSPEED);
            return true;
        }
        else{
            autobot.drive.resetMotors();
            return false;
        }
    }
}
