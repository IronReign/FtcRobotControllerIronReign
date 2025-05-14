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
    boolean turn=true;

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
        if(g1.dpad_up){
            if(robot.getUpMotor()<950){
                robot.setUpExtend(1000);
            }
            robot.wallGrab();

        }
        if(g1.dpad_down){
            prep();
            if(!robot.getMode()){
                robot.open();
                if(robot.getUpMotor()<1250){       //1600
                    robot.setUpExtend(1300);        //1700
                }
                if(robot.getUpMotor()>1280){
                    robot.setShoulder(1250);     //800
                }
                robot.setUpExtend(0);
            }
            robot.open();
            if(robot.getOutMotor()>300){          //if(robot.getOutExtend()>2600){
                robot.suck();
                robot.setTilt(760);     //750
            }
        }
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
        if(g1.a){
            robot.setClawP();
        }
        if(g1.x){
         //   if(robot.getMode()){
                robot.hookit();
                if(robot.getUpMotor()>2000){        //<--try new one
                //if(robot.getUpExtend()>2150){     //<--old one that works
                    robot.setShoulder(1270+420);        //check value make sure still good
                }
            // robot.setUpExtend(2250);
        }
        if(g1.y){
            if(robot.getMode()){
                robot.downHook();
            }else{
                robot.dunk();           //fix with new sample transfer presets
            }
        }
        if(g1.b){
            robot.setSuck(false);
        }


        if(robot.getlimit() && g2.guide){

            robot.calibrateUp();
        }
        if(g2.back){
            robot.recallibrate();
        }

        //TEST THIS THING
        if(g2.a){
            if(turn){
                robot.turnUntilDegreesIMU(0,1);
            }else{
                robot.turnUntilDegreesIMU(180,1);
            }
            turn=!turn;
        }
        //^^^TEST THIS THING

        if(g2.x){
            //SCOOCH SPECIMEN TO ONE SIDE OF BAR, MAKE SPACE
            robot.scooch();
        }
        if(g2.y){
            //NO PROJECTILE SPIT
            robot.plsnobad();
        }

        if(g1.guide){
            robot.resetDrive();
        }





        if(!robot.getlimit()){
            if(gamepad2.dpad_down && robot.getUpExtend()<3155){
                robot.addUpExtend(150);
            }//LOL PENIS - Poovid Pwyer
            if(gamepad2.dpad_up && robot.getUpExtend()>10){
                robot.addUpExtend(-150);
            }
            if(robot.getUpExtend()<0){
                robot.setUpExtend(0);
            }
            if(robot.getUpExtend()>3155){
                robot.setUpExtend(3150);
            }
        }else{
            if(gamepad2.dpad_down){
                robot.addUpExtend(150);
            }//LOL PENIS - Poovid Pwyer
            if(gamepad2.dpad_up){
                robot.addUpExtend(-150);
            }
        }


        if(gamepad2.dpad_right && robot.getOutExtend()>-20){
            robot.addOutExtend(-150);
        }
        if(gamepad2.dpad_left && robot.getOutExtend()<1080){        //&& robot.getOutExtend()<1950
            robot.addOutExtend(150);
        }
        if(robot.getOutExtend()>1080){
            robot.setOutExtend(1050);
        }
        if(robot.getOutExtend()<-20){
            robot.setOutExtend(-5);
        }

        if(gamepad1.left_bumper && robot.getShoulder()<1860){      //lim 1750
            robot.addShoulder(20);
        }
        if(gamepad1.right_bumper &&  robot.getShoulder()>790){
            robot.addShoulder(-20);
        }
        if(gamepad1.left_trigger>.3 &&robot.getTilt()<1280){
            robot.addTilt(20);
        }
        if(gamepad1.right_trigger>.3 && robot.getTilt()>740){
            robot.addTilt(-20);
        }



        if(g1.back){
            robot.mode();
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
    public void prep(){
        //robot.open();
        //robot.setUpExtend(350);
       // robot.setShoulder(950);     //930
        //robot.setTilt(970);
        robot.setOutPower(1);
        robot.setOutExtend(1050);
    }
}
