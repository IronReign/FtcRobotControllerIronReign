package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
public class Drone {
    private Servo droneLaunch = null;
    public static double RELEASE = 0.7;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public Drone(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void telemetryOutput()
    {
        telemetry.addData("Drone Launch Servo  \t", Utils.servoDenormalize(droneLaunch.getPosition()));
    }
    public void droneInit()
    {
        droneLaunch = this.hardwareMap.get(Servo.class, "droneLaunch");
    }
    public void droneRelease (boolean press)
    {
        if(press == true)
            droneLaunch.setPosition(RELEASE);
    }



}

