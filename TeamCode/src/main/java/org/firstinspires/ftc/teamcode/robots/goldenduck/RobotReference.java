package org.firstinspires.ftc.teamcode.robots.goldenduck;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robots.goldenduck.Railgun.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;



public class RobotReference {
    Telemetry telemetry;
    Railgun droneShot;
    public RobotReference(Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.telemetry=telemetry;
        droneShot = new Railgun(telemetry, hardwareMap);
    }

    public void robot(Telemetry telemetry, HardwareMap hardwareMap){
        droneShot = new Railgun(telemetry, hardwareMap);
        this.telemetry=telemetry;
    }
}
