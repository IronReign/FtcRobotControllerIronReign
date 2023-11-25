package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Robot {

    Telemetry telemetry;
        DriveTrain driveTrain;
        Drone droneLaunch;
        IntakeClaw claw;
        public Robot(Telemetry telemetry, HardwareMap hardwareMap)
        {
            this.telemetry=telemetry;
            driveTrain = new DriveTrain(telemetry, hardwareMap);
            droneLaunch = new Drone(telemetry, hardwareMap);
            claw = new IntakeClaw(telemetry, hardwareMap);
            motorInit();
            droneInit();
            intakeClawInit();

        }
        public void motorInit()
        {
            driveTrain.motorInit();

        }
        public void droneInit()
        {
            droneLaunch.droneInit();
        }
        public void intakeClawInit()
        {
            claw.intakeClawInit();
        }
}
