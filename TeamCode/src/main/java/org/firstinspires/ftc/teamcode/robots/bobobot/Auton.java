package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;

import java.util.LinkedList;
import java.util.Queue;

public class Auton {
    Queue<Assign> actions;
    Autobot autobot;
    Telemetry telemetry;
    public Auton(Autobot autobot, MultipleTelemetry telemetry){
        actions = new LinkedList<Assign>();
        this.autobot = autobot;
        this.telemetry = telemetry;
    }
    public void telemetryOutput(){
        telemetry.addData("Queue Size \t", acts());
        telemetry.addData("Error \t", getEachDelta() - autobot.drive.getMotorAvgPosition());
    }
    public boolean runBehaviors(){
        if(actions.size() > 0){
            if(!actions.peek().run()){
                actions.poll();
            }
            return true;
        }
        return false;
    }
    public double getEachDelta(){
        if (actions.size()>0)
            return actions.peek().getDelta();
        else return 0;
    }
    public void add(Assign assign){actions.offer(assign); }
    public int acts(){ return actions.size();}
    public boolean hasBehaviors()
    {
        return actions.size() > 0;
    }
    public void clearAuton () { actions.clear(); }
}
