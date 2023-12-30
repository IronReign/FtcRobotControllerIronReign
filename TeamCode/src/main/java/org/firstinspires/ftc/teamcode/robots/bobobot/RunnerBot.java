package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;

public class RunnerBot {
    public Subsystem[] subsystems;
    public DriveTrain driveTrain;
    public HardwareMap hardwareMap;
    Telemetry telemetry;
    public RunnerBot(MultipleTelemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        driveTrain = new DriveTrain(hardwareMap, this);
        subsystems = new Subsystem[]{driveTrain};
    }


}
