package org.firstinspires.ftc.teamcode.robots.bobobot.Bots;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RunnerBot {
    public DriveTrain driveTrain;

    public HardwareMap hardwareMap;
    private VoltageSensor batteryVoltageSensor;
    public RunnerBot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

}
