package org.firstinspires.ftc.teamcode.robots.goldenduck;

import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

public class Railgun {
    private Servo droneShot = null;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public Railgun(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        droneShot = this.hardwareMap.get(Servo.class, "servoRailgun");
    }
    public void telemetryOutput()
    {
        telemetry.addData("Drone Launch Servo  \t", Utils.servoDenormalize(droneShot.getPosition()));
    }

    public void droneGo (boolean press)
    {
        if(press == true)
            droneShot.setPosition(servoNormalize(1825));

    }

}