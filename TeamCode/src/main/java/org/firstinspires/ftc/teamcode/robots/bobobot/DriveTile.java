package org.firstinspires.ftc.teamcode.robots.bobobot;

class DriveTile extends Assign {
    private Autobot autobot;
    private double tiles;
    private double delta;
    private long oldTime = 0;
    // negative tiles means going backwards
    public DriveTile(Autobot autobot, double tiles){
        this.autobot = autobot;
        this.tiles = tiles;
        delta = autobot.drive.getMotorAvgPosition() + Math.abs(tiles*TICKSPERTILE);
    }
    @Override
    public boolean run(){
        if(autobot.drive.getMotorAvgPosition() < delta){
            autobot.drive.mechanumAuto(tiles/Math.abs(tiles)*MAXMOTORSPEED, 0, 0);
            return true;
        }
        else{
            if(oldTime == 0)
                oldTime = System.nanoTime();
            if(System.nanoTime() > oldTime+(TIMEBETWEENTASKS*NANOTOSECOND))
                return false;
            return true;
        }
    }
}
