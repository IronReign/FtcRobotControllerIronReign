package org.firstinspires.ftc.teamcode.robots.core;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.Map;

public class toDelete extends OpMode {
    public static double ONE_TILE_TIMER = 1.5;
    public static double TURN_TIMER = 1.5;
    Robot robot;
    HardwareMap hardwareMap;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        robot.init();
    }

    @Override
    public void loop() {
        robot.update(new Canvas());
        execute();
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }

    //forward one tile, turn 135, forward 1.25, extend arm, drop
    int autonIndex = 0;
    long autonTimer = 0;
    public void execute() {
        switch (autonIndex) {
           case 0:
               autonTimer = futureTime(ONE_TILE_TIMER);
               robot.mecanumDrive(1, 0, 0);
               autonIndex++;
                break;
            case 1:
                if(isPast(autonTimer)){
                    autonTimer = futureTime(TURN_TIMER);
                    robot.mecanumDrive(0, 0, 1);
                    autonIndex++;
                }
                break;
            case 2:
                if(isPast(autonTimer)){
                    autonTimer = futureTime(ONE_TILE_TIMER * 1.25);
                    robot.mecanumDrive(1, 0, 0);
                    autonIndex++;
                }
                break;
            case 3:
//                if (isPast(autonTimer)){
//                    robot.mecanumDrive(0, 0, 0);
//                    robot.shoulder
//                    autonIndex++;
//                }
                break;
        }
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        telemetry.addLine(telemetryName);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
        }

        telemetry.addLine();
    }
}
