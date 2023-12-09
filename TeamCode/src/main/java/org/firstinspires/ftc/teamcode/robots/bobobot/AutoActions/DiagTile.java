package org.firstinspires.ftc.teamcode.robots.bobobot.AutoActions;

import org.firstinspires.ftc.teamcode.robots.bobobot.Assign;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;

public class DiagTile extends Assign {
    private Autobot autobot;
    private double tiles;
    public DiagTile(Autobot autobot, double tiles){
        this.autobot = autobot;
        this.tiles = tiles;
        delta = autobot.drive.getMotorAvgPosition() + Math.abs(tiles*TICKSPERDIAG);
    }

    @Override
    public double getDelta(){return delta;}

    @Override
    public boolean run(){
        if(autobot.drive.getMotorAvgPosition() < delta){
            autobot.drive.mecanumAuto((0.5*Math.sqrt(2))*tiles/Math.abs(tiles)*MAXMOTORSPEED,(0.5*Math.sqrt(2))*tiles/Math.abs(tiles)*MAXMOTORSPEED , 0);
            return true;
        }
        else {

            autobot.drive.resetMotors();
            return false;
        }
    }

}
