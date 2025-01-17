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

//        rotate entire arm
        if(gamepad1.dpad_down ){//&& robot.getRotate()>10
            robot.tu(-20);
        }
        if(gamepad1.dpad_up ){
            robot.tu(20);
        }

        //ROTATION LIMITS
        if(robot.getRotate()<12){
            robot.setRotate(15);
        }
        if(robot.getRotate()>2260){
            robot.setRotate(2250);
        }

        //open and close claw
        if(g1.a) {
            robot.setClawP();
        }

        //extend linear slide
        if(gamepad1.left_bumper&&robot.getExtend()>=10) { //&&robot.getExtend()>=10
            robot.extend(-100);
        }


        if(robot.getRotate()>350){
            if(robot.getExtend()<5){
                robot.setExtend(10);
            }
        }
        if(robot.getRotate()<350&&robot.getRotate()>15
        ){
            if(robot.getExtend()<710){
                robot.setExtend(712);
            }
        }

        if(gamepad1.right_bumper && robot.getExtend()<8840) {//&& robot.getExtend()<7370+200
            robot.extend(100);
        }

        //EXTEND LIMITS
        if(robot.getRotate()>750){
            if(robot.getExtend()>8860){
                robot.setExtend(8850);
            }
        }else{
            if(robot.getExtend()>7112){
                robot.setExtend(7105);
            }
        }

        if(g1.left_trigger){
            robot.setDrive(.5*gamepad1.left_stick_y,.5*-gamepad1.left_stick_x,gamepad1.right_stick_x);
        }
        if(g1.right_trigger){
            robot.setDrive(gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);
        }
//        if(g1.a){
//            robot.suck();
//        }
//        if(gamepad1.b){
//            robot.setSlurp(true);
//        }else{
//            robot.setSlurp(false);
//        }
//        if(gamepad1.x){
//            robot.spit(true);
//        }else{
//            robot.spit(false);
//        }

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
