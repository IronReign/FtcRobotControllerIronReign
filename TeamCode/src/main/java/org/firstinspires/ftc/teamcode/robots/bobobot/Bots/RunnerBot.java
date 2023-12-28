package org.firstinspires.ftc.teamcode.robots.bobobot.Bots;

import static org.firstinspires.ftc.teamcode.robots.csbot.DriverControls.fieldOrientedDrive;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class RunnerBot {
    public Subsystem[] subsystems;
    public DriveTrain driveTrain;

    public HardwareMap hardwareMap;
    Telemetry telemetry;
    private VoltageSensor batteryVoltageSensor;
    public RunnerBot(MultipleTelemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        driveTrain = new DriveTrain(hardwareMap, this);
        subsystems = new Subsystem[]{driveTrain};
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


}
