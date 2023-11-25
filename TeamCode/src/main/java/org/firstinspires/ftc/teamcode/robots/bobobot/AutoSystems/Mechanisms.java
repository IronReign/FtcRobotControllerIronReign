package org.firstinspires.ftc.teamcode.robots.bobobot.AutoSystems;

import org.firstinspires.ftc.teamcode.robots.bobobot.Assign;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;

public class Mechanisms extends Assign {
    private Autobot autobot;
    private int instance;
    public Mechanisms(Autobot autobot, int instance){
        this.autobot = autobot;
        this.instance = instance;
        delta = 0;
    }
    @Override
    public double getDelta() {
        return delta;
    }
    @Override
    public boolean run(){
        if(instance == 1){
            autobot.grip.autoWristDown();
            return false;
        }
        else if(instance == 2){
            autobot.grip.autoOpen();
            return false;
        }
        else if(instance == 3){
            autobot.grip.autoClose();
            return false;
        }
        return false;
    }
}
