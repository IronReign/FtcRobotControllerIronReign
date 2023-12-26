package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.bobobot.AutoActions.DriveTile;
import org.firstinspires.ftc.teamcode.robots.bobobot.AutoActions.StrafeTile;
import org.firstinspires.ftc.teamcode.robots.bobobot.AutoActions.TurnTile;
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
    public void init_loop(){
        //autonomous.runBehaviors();
    }
    @Override
    public void start() {
        autonomous.add(new DriveTile(autobot,1.8));
    }

    @Override
    public void loop() {
        autobot.drive.update();
        autobot.grip.telemetryOutput();
        autonomous.telemetryOutput();
        autonomous.runBehaviors();
        dashTelemetry.update();
    }


    public void redBack(){
        autonomous.add(new TurnTile(autobot, -70));
        autonomous.add(new DriveTile(autobot, 2));
    }

    public void redFront(){
        autonomous.add(new DriveTile(autobot,1.1));
        autonomous.add(new TurnTile(autobot, -70));
        autonomous.add(new DriveTile(autobot, 3));
        autonomous.add(new TurnTile(autobot, -70));
        autonomous.add(new DriveTile(autobot,1));
        autonomous.add(new TurnTile(autobot, 70));
        autonomous.add(new DriveTile(autobot,1 ));
    }
    public void blueBack(){
        autonomous.add(new TurnTile(autobot, 70));
        autonomous.add(new DriveTile(autobot, 1.8));
    }

    public void blueFront(){
        autonomous.add(new DriveTile(autobot,1.1));
        autonomous.add(new TurnTile(autobot, 70));
        autonomous.add(new DriveTile(autobot, 3));
        autonomous.add(new TurnTile(autobot, 70));
        autonomous.add(new DriveTile(autobot,1));
        autonomous.add(new TurnTile(autobot, -70));
        autonomous.add(new DriveTile(autobot,1 ));
    }
}
