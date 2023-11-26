package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.bobobot.AutoSystems.Mechanisms;
import org.firstinspires.ftc.teamcode.robots.bobobot.AutoTiles.DriveTile;
import org.firstinspires.ftc.teamcode.robots.bobobot.AutoTiles.StrafeTile;
import org.firstinspires.ftc.teamcode.robots.bobobot.AutoTiles.TurnTile;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Autobot;

@Config("BoboAutoVariables")
@Autonomous(name = "BoboAuto")
public class BoboAuto extends OpMode {
    Autobot autobot;
    Auton autonomous;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        autobot = new Autobot(dashTelemetry, hardwareMap);
        autonomous = new Auton(autobot, dashTelemetry);
        dashTelemetry.setMsTransmissionInterval(25);
    }

    @Override
    public void start() {
        autonomous.add(new Mechanisms(autobot));
        autonomous.add(new DriveTile(autobot, 1));
        autonomous.add(new TurnTile(autobot, -45));
        autonomous.add(new DriveTile(autobot, 2));
        autonomous.add(new StrafeTile(autobot, -0.8));
    }
    @Override
    public void loop() {
        autobot.drive.telemetryOutput();
        autobot.grip.telemetryOutput();
        autonomous.telemetryOutput();
        autonomous.runBehaviors();
        dashTelemetry.update();
    }
}
