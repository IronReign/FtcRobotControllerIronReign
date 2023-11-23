package org.firstinspires.ftc.teamcode.robots.bobobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class TurnTile extends Assign{
    private Autobot autobot;
    private Telemetry telemetry;
    private double degrees;
    private double delta;

    public TurnTile(Autobot autobot, double degrees){
        this.telemetry = telemetry;
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
            autobot.drive.turnOffMotors();
            return false;
        }
    }
}
