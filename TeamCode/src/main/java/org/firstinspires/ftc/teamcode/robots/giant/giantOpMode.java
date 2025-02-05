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
    StickyGamepad g2=null;
    int dunk=0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1, gamepad2);
        g1=new StickyGamepad(gamepad1);
        g2=new StickyGamepad(gamepad2);
        robot.init();

    }

    @Override
    public void loop() {
        g1.update();
        g2.update();
        handleJoysticks(gamepad1, gamepad2);

        robot.update(new Canvas());
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }

    public void handleJoysticks(Gamepad gamepad, Gamepad gamepadtwo){
        //change gamemode transfer,

        if(g1.dpad_down){
            prep();
            if(robot.getOutExtend()>2600){
                robot.suck();
            }
        }
        if(g1.a){
            robot.setClawP();
        }


        if(g2.guide){
            robot.resetDrive();
        }

        if(g1.back){
            robot.mode();
        }
        if(g1.x){
            robot.hookit();
           // robot.setUpExtend(2250);
            if(robot.getUpExtend()>2150){
                robot.setShoulder(1270);        //1270
            }
            //robot.hookit();
        }
        if(g1.y){
            robot.downHook();
        }

//        if(gamepad2.x){
//            robot.
//            robot.setSlurp(false);
//        }else{
//
//        }

        if(g1.dpad_up){
            if(robot.getUpExtend()<650){
                robot.setUpExtend(700);
            }
            robot.wallGrab();

        }
//        if(g1.y){
//            if(!robot.isHook()){
//                robot.dunk();
//            }else {
//                if(robot.getUpExtend()<450){
//                    robot.setUpExtend(550);
//                }
//                robot.wallGrab();
//            }
//            //robot.transfer();
//
//        }
//        if(g2.b){
//            robot.hookit();
//        }
//        if(g2.a){
//            robot.downHook();
//        }

        if(gamepad2.dpad_down && robot.getUpExtend()<3155){
            robot.addUpExtend(150);
        }
        if(gamepad2.dpad_up && robot.getUpExtend()>10){
            robot.addUpExtend(-150);
        }
        if(robot.getUpExtend()<0){
            robot.setUpExtend(0);
        }
        if(robot.getUpExtend()>3155){
            robot.setUpExtend(3150);
        }
//        if(gamepad1.dpad_left){
//            robot.setOutExtend(-15);
//        }
//        if(gamepad1.dpad_right){
//            robot.setOutExtend(3350);
//        }

        if(gamepad2.dpad_left && robot.getOutExtend()>-20){
            robot.addOutExtend(-150);
        }
        if(gamepad2.dpad_right && robot.getOutExtend()<2000){
            robot.addOutExtend(150);
        }
        if(robot.getOutExtend()>2000){
            robot.setOutExtend(1990);
        }
        if(robot.getOutExtend()<-20){
            robot.setOutExtend(-5);
        }

        if(gamepad1.left_bumper && robot.getShoulder()<1860){      //lim 1750
            robot.addShoulder(20);
        }
        if(gamepad1.right_bumper ){ //&&  robot.getShoulder()>790
            robot.addShoulder(-20);
        }
        if(gamepad1.left_trigger>.3 &&robot.getTilt()<1280){
            robot.addTilt(20);
        }
        if(gamepad1.right_trigger>.3 && robot.getTilt()>740){
            robot.addTilt(-20);
        }

        if(g1.b){
            robot.setSuck(false);
        }


//        if(gamepad1.right_bumper && robot.getShoulder()<1860){      //lim 1750
//            robot.addShoulder(20);
//        }
//        if(gamepad1.left_bumper &&  robot.getShoulder()>790){
//            robot.addShoulder(-20);
//        }
//        if(gamepad1.right_trigger>.3 &&robot.getTilt()<1280){
//            robot.addTilt(20);
//        }
//        if(gamepad1.left_trigger>.3 && robot.getTilt()>740){
//            robot.addTilt(-20);
//        }


        robot.setDrive(gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);
//        if(g1.a) {
//            prep();
//            if(robot.getOutExtend()>2600){
//                robot.suck();
//            }
//
//        }


        if(gamepad1.dpad_left){
            robot.setSlurp(true);
        }else{
            robot.setSlurp(false);
        }
        if(gamepad1.dpad_right){
            robot.setTilt(970);
            robot.spit(true);
        }else{
            robot.spit(false);
        }


//        if(g1.guide){
//            robot.resetDrive();
//        }

    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        telemetry.addLine(telemetryName);
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
        }
        telemetry.addLine();
    }
    public void prep(){
        robot.open();
        robot.setUpExtend(350);
        robot.setShoulder(950);     //930
        robot.setTilt(970);
        robot.setOutExtend(2750);
    }
}
