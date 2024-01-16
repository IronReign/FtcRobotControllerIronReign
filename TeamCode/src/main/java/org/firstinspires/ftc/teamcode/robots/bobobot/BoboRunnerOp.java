package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.Toggle;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;

import java.util.Map;

@TeleOp(name = "BoboRunner")
public class BoboRunnerOp extends OpMode {
    public static RunnerBot runnerBot;
    //IMU imu;
    Toggle toggle;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;
    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry,hardwareMap);
        toggle = new Toggle(gamepad1, gamepad2);
        //imu = new IMU(dashTelemetry, hardwareMap);
        dashTelemetry.setMsTransmissionInterval(25);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void loop(){
        toggle.gamepadUpdate();
        toggle.toggleSpeedMode(); //Sticky Gamepad Controls and Update
        toggle.intake();
        toggle.drone();
        runnerBot.driveTrain.drive(gamepad1.left_stick_x*spd(), gamepad1.left_stick_y*spd(), gamepad1.right_stick_x*spd());
        //imu.telemetryOutput();
        update();
        dashTelemetry.update();

    }

    private void update(){
        TelemetryPacket packet = new TelemetryPacket();

        for(TelemetryProvider telemetryProvider: runnerBot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(false), telemetryProvider.getTelemetryName(), packet);
        runnerBot.update(packet.fieldOverlay());
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    public void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet){
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);
        packet.addLine("");

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }

    public double spd(){
        return runnerBot.driveTrain.getRobotSpeed();
    }
    //Method for getting the robot's speed constant (check robotSpeed in DriveTrain class)
}
