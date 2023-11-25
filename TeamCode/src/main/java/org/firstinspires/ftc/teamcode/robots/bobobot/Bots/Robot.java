package org.firstinspires.ftc.teamcode.robots.bobobot.Bots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleSystems.Drone;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleSystems.IntakeClaw;


public class Robot {
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/bobobot/Robot.java

    Telemetry telemetry;
        DriveTrain driveTrain;
        Drone droneLaunch;
        IntakeClaw claw;
=======
        Telemetry telemetry;
        public DriveTrain driveTrain;
        public Drone droneLaunch;
        public IntakeClaw claw;
>>>>>>> upstream/main:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robots/bobobot/Bots/Robot.java
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
