package org.firstinspires.ftc.teamcode.robots.bobobot.Bots;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems.OpDrive;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems.Drone;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems.IntakeClaw;


public class Robot {
        Telemetry telemetry;
        public OpDrive opDrive;
        public Drone droneLaunch;
        public IntakeClaw claw;

        public Robot(MultipleTelemetry telemetry, HardwareMap hardwareMap)
        {
            this.telemetry = telemetry;
            opDrive = new OpDrive(telemetry, hardwareMap);
            droneLaunch = new Drone(telemetry, hardwareMap);
            claw = new IntakeClaw(telemetry, hardwareMap);

            motorInit();
            droneInit();
            intakeClawInit();
        }



    public void motorInit() {
        opDrive.motorInit();}
    public void droneInit()
        {
            droneLaunch.droneInit();
        }
    public void intakeClawInit()
        {
            claw.intakeClawInit();
        }
}
