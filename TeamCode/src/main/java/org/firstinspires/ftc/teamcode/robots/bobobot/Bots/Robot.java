package org.firstinspires.ftc.teamcode.robots.bobobot.Bots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.bobobot.IMU;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems.Drone;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems.IntakeClaw;


public class Robot {
        Telemetry telemetry;
        public DriveTrain driveTrain;
        public Drone droneLaunch;
        public IntakeClaw claw;

        public Robot(MultipleTelemetry telemetry, HardwareMap hardwareMap)
        {
            this.telemetry = telemetry;
            driveTrain = new DriveTrain(telemetry, hardwareMap);
            droneLaunch = new Drone(telemetry, hardwareMap);
            claw = new IntakeClaw(telemetry, hardwareMap);

            motorInit();
            droneInit();
            intakeClawInit();
        }



    public void motorInit() {driveTrain.motorInit();}
    public void droneInit()
        {
            droneLaunch.droneInit();
        }
    public void intakeClawInit()
        {
            claw.intakeClawInit();
        }
}
