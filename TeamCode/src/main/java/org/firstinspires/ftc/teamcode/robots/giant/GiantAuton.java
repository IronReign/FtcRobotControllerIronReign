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
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;


import java.util.Map;


@Autonomous(name = "auton SPECIMEN")
@Config(value = "auton SPECIMEN")
public class GiantAuton extends OpMode {


    public static double FORWARD=1.21;
    public static double BUFFER=.3;
    public static double LONGBUFFER= 1.9;        //2.2
    public static double BACKWARD=.255;     //.255


    Robot robot;
    StickyGamepad g1=null;
    StickyGamepad g2=null;
    //HardwareMap hardwareMap;


    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1,gamepad2);
        robot.init();
        //  robot.setRotate(950);
    }
    @Override
    public void init_loop(){
        g1=new StickyGamepad(gamepad1);
        g2=new StickyGamepad(gamepad2);
        g1.update();
        g2.update();
        if(g1.guide){
            robot.changeAlly();
        }
        robot.update(new Canvas());
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }


    @Override
    public void loop() {
        robot.update(new Canvas());
        execute();
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }


    //forward vertical for one tile practice    vertical=4000;

    //forward one tile, turn 135, forward 1.25, extend arm, drop
    int autonIndex = 0;
    long autonTimer = 0;
    public void execute() {
        switch (autonIndex) {
            case 0:
                robot.close();
                robot.setUpExtend(2250);        //2250
                autonIndex++;
                break;
            case 1:
                if(robot.getUpExtend()>2000){
                    robot.resetDrive();
                    robot.setShoulder(1250);        //1350
                }
                if(robot.getShoulder()>1230){
                    autonIndex++;
                }

                break;
            case 2:
              //  robot.driveDistance(13,.5);
                if (robot.driveDistance(15.5,.6)){    //17.5    .5
                    autonTimer=futureTime(.5);
                    autonIndex++;
                }
                break;

            case 3:
                if(isPast(autonTimer)){
                    robot.downHook();
                    autonTimer=futureTime(.6);
                    autonIndex++;
                }

                break;
            case 4:
                if(isPast(autonTimer)){
                    robot.open();
                    robot.wallGrab();
                    autonIndex++;
                }
                break;
            case 5:
                if(robot.driveDistance(35,.8)){
                    autonTimer=futureTime(.2);
                    autonIndex++;
                }
                break;
            case 6:
                if(isPast(autonTimer)){
                    autonIndex++;
                }
                break;
            case 7:
                if(robot.strafe(43,.9,true)){    //         closer??
                    robot.resetDrive();
                    robot.setDrive(0,0,.25);
//                    robot.setTilt(740);
//                    robot.setOutExtend(1150);
                    ////robot.setOutExtend(2380);
//                    robot.suck();
                    autonIndex++;
                }
                break;
            case 8:

                if(robot.getHor()<=(-3780) &&robot.getVert()<=(-7180)){      //robot.getHor()<=(-3780) &&robot.getVert()<=(-7180)
                    robot.resetDrive();
                    robot.setDrive(0,0,-.5);    //930
                    autonTimer=futureTime(.05);            //.065
                    //autonTimer=futureTime(1.5);
                    autonIndex++;
                }
                break;
            case 9:
                if(isPast(autonTimer)){
                    autonTimer=futureTime(1.5);
                    robot.setDrive(0,0,0);
                    autonIndex++;
                }
                break;
            case 10:
                if(isPast(autonTimer)){
                    robot.resetDrive();
                    robot.setDrive(.45,0,0);        //.45
                    autonIndex++;
                }
                break;
            case 11:
                if(robot.getVert()<-1778){      //-1760     //-1650
                    robot.setDrive(0,0,0);
                    autonTimer=futureTime(.02);
                    autonIndex++;
                }
                break;
            case 12:
                if(isPast(autonTimer)){
                    robot.close();
                    autonTimer=futureTime(.7);
                    autonIndex++;
                }
                break;
            case 13:
                if(isPast(autonTimer)){
                    robot.hookit();
                    robot.setUpExtend(2265);
                    autonIndex++;
                }
                break;
            case 14:
                if(robot.getUpExtend()>150){
                    robot.resetDrive();
                    robot.setDrive(-.8,0,0);
                    autonIndex++;
                }
                break;
            case 15:
                if(robot.getVert()>2100){       //3200
                    robot.setShoulder(1250);
                    robot.resetDrive();
                    robot.setDrive(0,1,0);
                    autonIndex++;
                }
                break;
            case 16:
                if(robot.getHor()<-13100){
                //if(robot.strafe(130,1,false)){
                    robot.resetDrive();
                    robot.setDrive(0,0,.25);
                    autonIndex++;
                }
            case 17:
                if(robot.getHor()<=(-3850) &&robot.getVert()<=(-7250) ){         //robot.getHor()>=2270 &&robot.getVert()>=6270
                    robot.setDrive(0,0,0);    //930
                    autonIndex++;
                }
                break;
            case 18:
                if(isPast(autonTimer)){
                    robot.setDrive(-.8,0,0);
                    autonTimer=futureTime(.15);
                    autonIndex++;
                }
                break;
            case 19:
                if(isPast(autonTimer)){
                    autonIndex++;
                }
                break;

            case 20:
                if (robot.driveDistance(9,.6)){    //17.5    .5
                    autonTimer=futureTime(.5);
                    autonIndex++;
                }
                break;
            case 21:
                if(isPast(autonTimer)){
                    robot.downHook();
                    autonTimer=futureTime(.6);
                    autonIndex++;
                }
                break;
            case 22:
                if(isPast(autonTimer)){
                    robot.open();
                    autonTimer=futureTime(.02);
                    autonIndex++;
                }
                break;
            case 23:
                if(isPast(autonTimer)){
                    autonTimer=futureTime(.5);
                    robot.setDrive(-.8,0,0);
                    robot.setUpExtend(1800);
                    autonIndex++;
                }
                break;
            case 24:
                if(isPast(autonTimer)){
                    robot.setDrive(0,0,0);
                    robot.setShoulder(750);
                    robot.setUpExtend(0);
                    autonIndex++;
                }
                break;

//            case 23:
//                if(isPast(autonTimer)){
//                    robot.hookit();
//                    robot.setShoulder(1250);
//                    autonIndex++;
//                }
//                break;
//            case 24:
//
//
//
//                if(isPast(autonTimer)){
//                    robot.open();
//                    autonIndex++;
//                }
//                break;


                ////SPIT ROUTINE????
//            if(robot.getOutExtend()>950){
//                autonTimer=futureTime(1);
//                robot.spit(true);
//                autonIndex++;
//            }
//            break;
//            case 15:
//                if(isPast(autonTimer)){
//                    robot.spit(false);
//                    robot.setOutExtend(50);
////                    robot.setOutExtend(100);
////                    robot.setShoulder(1450);
//                    robot.resetDrive();
//                    robot.setDrive(0,0,-.8);
//                    robot.setOutPower(.7);
//                    autonIndex++;
//                }
//                break;



                ////GATHER MORE BLOCKS CODE????


//            case 17:
//                //if(robot.strafe(15,.3,false)){
//                robot.setDrive(0,-.7,0);
//                autonTimer=futureTime(1.2);
//                autonIndex++;
//            //    }
//                break;
//            case 18:
//                if(isPast(autonTimer)){
//                    robot.setOutPower(1);
//                    robot.resetDrive();
//                    robot.setDrive(0,0,-.8);        //speed up?
//                    autonIndex++;
//                }
//                break;
//            case 19:
//                if(robot.getHor()>=2270 &&robot.getVert()>=6270){           //robot.getHor()>=3550 &&robot.getVert()>=7550
//                    robot.setDrive(0,0,0);
//                    robot.setOutExtend(1400);
//                    autonIndex++;
//                }
//                break;
//            case 20:
//                if(robot.getOutExtend()>1350){
//                    autonTimer=futureTime(1);
//                    robot.spit(true);
//                    autonIndex++;
//                }
//                break;
//            case 21:
//                break;


            //IDK????

//            case 16:
//                if(robot.getOutExtend()<300){
//                    robot.setDrive(0,0,-.5);
//                    autonIndex++;
//                }
//                break;
//            case 17:
//                if(robot.getHor()>=3700 &&robot.getVert()>=7700){
//                    robot.setDrive(0,0,0);
//                    //robot.prep();
//                    autonIndex++;
//                }
//                break;
//            case 18:
//                if(robot.strafe(16,.5)){
//                    robot.prep();
//                    autonIndex++;
//                }
//                break;
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
