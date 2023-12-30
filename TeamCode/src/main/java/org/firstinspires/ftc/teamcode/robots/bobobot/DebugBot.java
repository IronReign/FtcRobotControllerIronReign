package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;

public class DebugBot {
    public MotorDebug motorDebug;
    public HardwareMap hardwareMap;
    Telemetry telemetry;

    public DebugBot(MultipleTelemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        motorDebug = new MotorDebug(telemetry, hardwareMap);
    }
}
