package org.firstinspires.ftc.teamcode.robots.bobobot.AutoSystems;

import org.firstinspires.ftc.teamcode.robots.bobobot.Assign;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;

public class Mechanisms extends Assign {
    private Autobot autobot;
    private int instance;
    public Mechanisms(Autobot autobot){
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
        autobot.grip.autoWristDown();
        autobot.grip.autoOpen();
        autobot.grip.autoClose();
        return false;
    }
}
