package org.firstinspires.ftc.teamcode.robots.bobobot.Bots;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.AutoSystems.AutoClaw;
import org.firstinspires.ftc.teamcode.robots.bobobot.AutoSystems.AutoDrive;

public class Autobot {
    Telemetry telemetry;
    public AutoDrive drive;

    public AutoClaw grip;

    public Autobot(MultipleTelemetry telemetry, HardwareMap hardwareMap)
    {
        this.telemetry=telemetry;
        drive = new AutoDrive(telemetry, hardwareMap);
        grip = new AutoClaw(telemetry, hardwareMap);

        driveInit();
        gripInit();
    }
    public void driveInit() { drive.driveInit(); }
    public void gripInit() {grip.gripInit();}
}
