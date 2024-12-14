package org.firstinspires.ftc.teamcode.robots.giant;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.Map;

@Autonomous(name = "GIANTAUTON")
@Config(value = "GiantAuton")
public class GiantAuton extends OpMode {

    public static double FORWARD=.69;
    public static double L=.91;    //.91
    public static double TURN_FORWARD_1 = .6;
    public static double BACK=.3;
    public static double TIME=.1;

            ; //experiment and fix
    Robot robot;
    //HardwareMap hardwareMap;

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
                autonTimer = futureTime(L);
                robot.grabBlock();
                robot.setDrive(-.5, 0, 0);
                robot.setShoulderSpeed(1);
                autonIndex++;
                break;
            case 1:
                if(isPast(autonTimer)){
                   // autonTimer = futureTime(TURN_TIMER);
                    robot.setDrive(0, 0, 0);
                    robot.rotateUpBar();;
                    autonIndex++;
                }
                break;
            case 2:
                if(robot.getRotate()>1390){
                    autonTimer = futureTime(TURN_FORWARD_1);
                    robot.unstickArm();

                    autonIndex++;
                }
                break;
            case 3:
                if(robot.getExtend()>7750){
                    robot.reach();

                    autonIndex++;
                }
                break;
            case 4:
                if(isPast(autonTimer)) {
                    robot.aim();
                    autonIndex++;
                }
                break;
            case 5:
                if(robot.getRotate()<1280) {
                    autonTimer=futureTime(TIME);
                    robot.pull();
                    autonIndex++;
                }
                break;
            case 6:
                if(robot.getExtend()<4760) {
                    autonTimer=futureTime(BACK);
                    robot.dropBlock();
                    robot.setDrive(.6,0,0);
                    autonIndex++;
                }
                break;
            case 7:
                if(isPast(autonTimer)) {
                    autonTimer=futureTime(TIME);
                    robot.setDrive(0,0,0);
                    robot.resetE();
                    autonIndex++;
                }
            case 8:
                if(isPast(autonTimer)){
                    robot.setShoulderSpeed(.4);
                    robot.setRotate(5);
                    autonIndex++;
                }

                break;
            case 9:
                if(robot.getRotate()<10){
                    robot.setShoulderSpeed(1);
                }

                break;

       }
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        telemetry.addLine(telemetryName);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
            //telemetry.addLine(""+autonIndex);
        }

        telemetry.addLine();
    }
}