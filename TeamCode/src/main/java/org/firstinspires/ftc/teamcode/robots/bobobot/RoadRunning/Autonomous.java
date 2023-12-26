package org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning;


import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value= "BobotAutonomous")
public class Autonomous {
    private RunnerBot runnerBot;
    private HardwareMap hardwareMap;

    private Action
    frontRed, backRed,
    frontBlue, backBlue;

    public Autonomous(RunnerBot runnerBot){
        this.runnerBot = runnerBot;
        this.hardwareMap = runnerBot.hardwareMap;
    }


}
