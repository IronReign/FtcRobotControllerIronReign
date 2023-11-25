package org.firstinspires.ftc.teamcode.robots.bobobot.AutoTiles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.Assign;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;

public class StrafeTile extends Assign {
    private Autobot autobot;
    private double tiles;
    @Override
    public double getDelta() {
        return delta;
    }
    //private double delta;
    public StrafeTile (Autobot autobot, double tiles){
        this.autobot = autobot;
        this.tiles = tiles;
        delta = autobot.drive.getMotorAvgPosition() + Math.abs(tiles*STRAFETICKSPERTILE);
    }

    @Override
    public boolean run(){
        if(autobot.drive.getMotorAvgPosition() < delta){
            autobot.drive.mecanumAuto(0, tiles/Math.abs(tiles)*MAXMOTORSPEED, 0);
            return true;
        }
        else{
            autobot.drive.resetMotors();
            return false;
        }
    }


}
