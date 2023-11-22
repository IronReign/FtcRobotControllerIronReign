package org.firstinspires.ftc.teamcode.robots.bobobot;

import java.util.LinkedList;
import java.util.Queue;

public class Auton {
    Queue<Assign> actions;
    Autobot autobot;
    public Auton(Autobot autobot){
        actions = new LinkedList<Assign>();
        this.autobot = autobot;
    }
    public boolean runBehaviors(){
        if(actions.size() > 0){
            if(!actions.peek().run()){
                actions.poll();
                autobot.drive.turnOffMotors();
            }
            return true;
        }
        return false;
    }
    public void add(Assign assign){actions.offer(assign); }
    public boolean hasBehaviors()
    {
        return actions.size() > 0;
    }
    public void clearAuton () { actions.clear(); }
}
