package org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.bobobot.IMU;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems.Toggle;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;

import java.util.Map;

@TeleOp(name = "BoboRunner")
public class BoboRunnerOp extends OpMode {
    public static RunnerBot runnerBot;
    IMU imu;
    Toggle toggle;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;

    private Action
            frontRed, backRed,
            frontBlue, backBlue;
    @Override
    public void init(){
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        runnerBot = new RunnerBot(dashTelemetry,hardwareMap);
        toggle = new Toggle(gamepad1);
        imu = new IMU(dashTelemetry, hardwareMap);
        dashTelemetry.setMsTransmissionInterval(25);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void loop(){
        toggle.gamepadUpdate();
        toggle.toggleSpeedMode();
        runnerBot.driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        imu.telemetryOutput();
        update();
        dashTelemetry.update();

    }
    private void update(){
        TelemetryPacket packet = new TelemetryPacket();

        for(TelemetryProvider telemetryProvider: runnerBot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(false), telemetryProvider.getTelemetryName(), packet);
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    public void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet){
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }
}
