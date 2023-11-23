package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config("BoboAutoVariables")
@Autonomous(name = "BoboAuto")
public class BoboAuto extends OpMode {

    Autobot autobot;
    Auton autonomous;
//    boolean ranOnce;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        autobot = new Autobot(dashTelemetry, hardwareMap);
        autonomous = new Auton(autobot);

        //dashboard.setTelemetryTransmissionInterval(25);

        dashTelemetry.setMsTransmissionInterval(25);
    }

    @Override
    public void start() {
        autonomous.add(new DriveTile(autobot, 1));
        autonomous.add(new TurnTile(autobot, 90));
    }
    @Override
    public void loop() {

        autobot.drive.telemetryOutput();
        autobot.grip.autoWrist();
        autonomous.runBehaviors();
        dashTelemetry.update();
    }
}
