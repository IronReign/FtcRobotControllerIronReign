package org.firstinspires.ftc.teamcode.robots.giant;

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
        //rotate entire arm
        if(gamepad1.dpad_down){
            robot.tu(-20);
        }
        if(gamepad1.dpad_up ){
            robot.tu(20);
        }


        //open and close claw
        if(g1.a) {
            robot.grabBlock();
        }
        if(g1.b) {
            robot.dropBlock();
        }

        //extend linear slide
        if(gamepad1.left_bumper &&robot.getExtend()>=10) {
            robot.extend(-50);
        }

        if(gamepad1.right_bumper && robot.getExtend()<=(9040-2110)) {//&&robot.getExtend()<7150
            robot.extend(50);
        }
        if(robot.getExtend()<=10)
        {
            robot.setExtend(12);
        }
        if(robot.getRotate()<40){
            robot.setRotate(50);
        }
        if(robot.getRotate()>2220){
            robot.setRotate(2150);
        }

        if(robot.getRotate()<700&&robot.getRotate()>2200) {
            if(robot.getExtend()>=5520)
            {
                robot.setExtend(5500);
            }
        }
        else{
            if(robot.getExtend()>=(9050-2110)){
                robot.setExtend(9030-2110);
            }
        }
        if(robot.getExtend()>=(9200-2110))
        {
            robot.setExtend(9100-2110);
        }
        if(g1.dpad_left) {
            robot.doit();
        }
        if(g1.dpad_right) {
            robot.attatch();
        }
        if(g1.guide)
        {
            robot.setStop(true);
        }


        robot.setDrive(gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x);

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
