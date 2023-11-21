package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "BoboAuto")
public class BoboAuto extends OpMode {

    Autobot autobot;
    Auton autonomous;
//    boolean ranOnce;
    FtcDashboard dashboard;
    @Override
    public void init() {
        autobot = new Autobot(telemetry, hardwareMap);
        autonomous = new Auton(autobot);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry.setMsTransmissionInterval(25);
    }

    boolean didDriveTile = false;
    @Override
    public void loop() {
        autobot.drive.telemetryOutput();
        autobot.grip.autoWrist();
        autonomous.add(new DriveTile(autobot, 1));
    }
}
