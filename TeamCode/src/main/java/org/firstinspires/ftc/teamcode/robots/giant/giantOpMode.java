package org.firstinspires.ftc.teamcode.robots.giant;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;


import java.util.LinkedHashMap;
import java.util.Map;
@TeleOp(name="giant mode", group="game")
public class giantOpMode extends OpMode {
    Robot robot;
    StickyGamepad g1=null;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        g1=new StickyGamepad(gamepad1);
        robot.init();

    }

    @Override
    public void loop() {
        g1.update();
        handleJoysticks(gamepad1);

        robot.update(new Canvas());
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }

    public void handleJoysticks(Gamepad gamepad){
        //change gamemode transfer,
        if(g1.back){
            robot.mode();
        }
        if(g1.y) {
            robot.setClawP();
           // robot.transfer();     <-- test this

        }

        if(gamepad1.dpad_up && robot.getUpExtend()<3150){
            robot.addUpExtend(150);
        }
        if(gamepad1.dpad_down && robot.getUpExtend()>10){
            robot.addUpExtend(-150);
        }
        if(robot.getUpExtend()<0){
            robot.setUpExtend(0);
        }
//        if(gamepad1.dpad_left){
//            robot.setOutExtend(-15);
//        }
//        if(gamepad1.dpad_right){
//            robot.setOutExtend(3350);
//        }

        if(gamepad1.dpad_left && robot.getOutExtend()>-150){
            robot.addOutExtend(-150);
        }
        if(gamepad1.dpad_right && robot.getOutExtend()<2750){
            robot.addOutExtend(150);
        }

        if(gamepad1.right_bumper){
            robot.addShoulder(20);
        }
        if(gamepad1.left_bumper){
            robot.addShoulder(-20);
        }
        if(gamepad1.right_trigger>.3){
            robot.addTilt(20);
        }
        if(gamepad1.left_trigger>.3){
            robot.addTilt(-20);
        }


        robot.setDrive(gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);
        if(g1.a) {
            robot.suck();
        }

        if(g1.x){
            robot.setSuck(false);
        }
        if(gamepad1.b){
            robot.setSlurp(true);
        }else{
            robot.setSlurp(false);
        }
        if(gamepad1.b){
            robot.spit(true);
        }else{
            robot.spit(false);
        }


        if(g1.guide){
            robot.resetDrive();
        }

        robot.setDrive(gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);
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
