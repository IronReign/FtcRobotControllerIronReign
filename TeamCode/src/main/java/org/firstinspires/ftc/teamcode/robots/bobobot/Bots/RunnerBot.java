package org.firstinspires.ftc.teamcode.robots.bobobot.Bots;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning.DriveTrain;

public class RunnerBot {
    public DriveTrain driveTrain;

    public HardwareMap hardwareMap;
    Telemetry telemetry;
    private VoltageSensor batteryVoltageSensor;
    public RunnerBot(MultipleTelemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        driveTrain = new DriveTrain(hardwareMap, this);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

}
