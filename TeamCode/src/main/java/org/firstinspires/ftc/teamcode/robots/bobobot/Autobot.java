package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Autobot {
    Telemetry telemetry;
    AutoDrive drive;
    AutoClaw grip;

    public Autobot(Telemetry telemetry, HardwareMap hardwareMap)
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
