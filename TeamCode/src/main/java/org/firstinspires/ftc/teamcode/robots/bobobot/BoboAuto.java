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

    @Override
    public void start() {
        autonomous.add(new DriveTile(autobot, 2, telemetry));
    }
    @Override
    public void loop() {
        autobot.drive.telemetryOutput();
        autobot.grip.autoWrist();
//        autobot.DriveTile.telemetryOutput();
        autonomous.runBehaviors();

    }
}
