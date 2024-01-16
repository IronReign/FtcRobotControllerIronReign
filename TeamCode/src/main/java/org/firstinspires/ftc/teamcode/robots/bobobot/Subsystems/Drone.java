package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

import java.util.HashMap;
import java.util.Map;

public class Drone implements Subsystem {
    private Servo droneLaunch = null;
    public static double RELEASE = 0.7;
    public RunnerBot runnerBot;
    private HardwareMap hardwareMap;
    public Drone(HardwareMap hardwareMap, RunnerBot runnerBot) {
        this.hardwareMap = hardwareMap;
        this.runnerBot = runnerBot;
        droneLaunch = this.hardwareMap.get(Servo.class, "droneLaunch");
    }

    public void droneRelease() {
        droneLaunch.setPosition(RELEASE);
    }
    @Override
    public Map<String,Object> getTelemetry(boolean debug){
        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("Drone Launch Servo  \t", Utils.servoDenormalize(droneLaunch.getPosition()));
        return telemetryMap;
    }
    @Override
    public String getTelemetryName(){
        return "DRONE";
    }
    @Override
    public void update(Canvas fieldOverlay){

    }
    @Override
    public void stop(){

    }


}

