package org.firstinspires.ftc.teamcode.robots.bobobot.Bots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.r2v2.vision.Target;

import java.util.ArrayList;
import java.util.List;

public class RunnerBot {
    public DriveTrain driveTrain;

    public HardwareMap hardwareMap;
    private VoltageSensor batteryVoltageSensor;
    public RunnerBot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

}
