package org.firstinspires.ftc.teamcode.robots.bobobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class DriveTile extends Assign {
    private Autobot autobot;
    private Telemetry telemetry;
    private double tiles;
    private double delta;
    private long oldTime = 0;
    // negative tiles means going backwards
    public DriveTile(Autobot autobot, double tiles, Telemetry telemetry){
        this.telemetry = telemetry;
        this.autobot = autobot;
        this.tiles = tiles;
        delta = autobot.drive.getMotorAvgPosition() + Math.abs(tiles*TICKSPERTILE);
    }
    @Override
    public boolean run(){
        if(autobot.drive.getMotorAvgPosition() < delta){
            autobot.drive.mecanumAuto(tiles/Math.abs(tiles)*MAXMOTORSPEED, 0, 0);
            return true;
        }
        else{
            autobot.drive.turnOffMotors();
//            if(oldTime == 0)
//                oldTime = System.nanoTime();
//            if(System.nanoTime() > oldTime+(TIMEBETWEENTASKS*NANOTOSECOND))
//                return false;

            return false;
        }
    }
    public void telemetryOutput(){
        telemetry.addData("Error \t", delta - autobot.drive.getMotorAvgPosition());
    }
}
