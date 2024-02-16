package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.IMU;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.Toggle;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;

import java.util.Map;

@TeleOp(name="BoboDebugOpMode", group="Challenge")
public class BoboDebugOp extends OpMode {
    public static DebugBot debugbot;
    public static RunnerBot runnerBot;
    public static boolean turn180,
            turn90,
            turn0 = false;
    IMU imu;
    Toggle toggle;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        debugbot = new DebugBot(dashTelemetry, hardwareMap);
        toggle = new Toggle(gamepad1, gamepad2, runnerBot);
        imu = new IMU(dashTelemetry, hardwareMap);
        dashTelemetry.setMsTransmissionInterval(25);
    }

    @Override
    public void loop() {
        toggle.gamepadUpdate();
        toggle.runTest();
        toggle.turnTest1();
        toggle.turnTest2();
        toggle.turnTest3();
        if(turn90 && debugbot.driveTrain.turnUntilDegreesIMU(90, .8))
            turn90 = false;
        if(turn180 && debugbot.driveTrain.turnUntilDegreesIMU(180,.8))
            turn180 = false;
        if(turn0 && debugbot.driveTrain.turnUntilDegreesIMU(0,.8))
            turn0 = false;
        telemetry.addData("Heading RR \t", debugbot.driveTrain.pose.heading.log());
        telemetry.addData("Heading IMU \t", debugbot.driveTrain.imuAngle);
        telemetry.addData("Heading IMU Pitch \t", debugbot.driveTrain.imuAngleP);
        telemetry.addData("Heading IMU Roll \t", debugbot.driveTrain.imuAngleR);
        telemetry.addData("Target Heading \t", debugbot.driveTrain.targetHeading);
        telemetry.addData("Angular Error \t", debugbot.driveTrain.getTargetHeading()- debugbot.driveTrain.getImuAngle());
        telemetry.addData("% Error \t", 100 * Math.abs((debugbot.driveTrain.getImuAngle() - debugbot.driveTrain.getTargetHeading())/debugbot.driveTrain.getTargetHeading()));
        telemetry.addData("Turn 0? \t", turn0);
        telemetry.addData("Turn 90? \t", turn90);
        telemetry.addData("Turn 180 \t", turn180);
        update();
        dashTelemetry.update();

    }

    private void update(){
        TelemetryPacket packet = new TelemetryPacket();

        for(TelemetryProvider telemetryProvider: debugbot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(false), telemetryProvider.getTelemetryName(), packet);
        debugbot.update(packet.fieldOverlay());
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


}
