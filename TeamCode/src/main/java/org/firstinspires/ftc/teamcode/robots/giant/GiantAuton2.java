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


@Autonomous(name = "auton SAMPLE")
@Config(value = "auton SAMPLE")
public class GiantAuton2 extends OpMode {


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
                robot.setUpExtend(3150);
                autonIndex++;
                break;
            case 1:
                if(robot.getUpExtend()>2000){
                    robot.setShoulder(1730);
                    autonIndex++;
                }
                break;
            case 2:
                if(robot.getShoulder()>1680){
                    robot.resetDrive();
                    robot.setDrive(.5,0,0);
                    autonIndex++;
                }
                break;
            case 3:
                if(robot.getVert()<-5050){
                    robot.setDrive(0,0,0);
                    autonTimer=futureTime(.2);
                    autonIndex++;
                }
                break;
            case 4:
                if(isPast(autonTimer)){
                    robot.open();
                    autonTimer=futureTime(.2);
                    autonIndex++;
                }
                break;
            case 5:
                if(isPast(autonTimer)){
                    autonTimer=futureTime(.7);
                    robot.setDrive(-.7,0,0);
                    autonIndex++;
                }
                break;
            case 6:
                if(isPast(autonTimer)){
                    robot.setDrive(0,0,0);
                    robot.setShoulder(750);
                    robot.setUpExtend(0);
                    autonTimer=futureTime(.02);
                    autonIndex++;
                }
                break;
//            case 7:
//                if(isPast(autonTimer)){
//                    robot.resetDrive();
//                    robot.setDrive(0,0,-.25);
//                    autonIndex++;
//                }
//                break;
//            case 8:
//                if(robot.getHor()>1700 && robot.getVert()>3900){
//                    robot.setDrive(0,0,0);
//                    robot.setTilt(970);
//                    robot.setOutExtend(1800);
//
//                    robot.suck();
//                    autonTimer=futureTime(.02);
//                    autonIndex++;
//                }
//                break;
//            case 9:
//                if(isPast(autonTimer)){
//                    autonTimer=futureTime(.5);
//                    robot.setDrive(0,-.2,0);
//                    autonIndex++;
//                }
//                break;
//            case 10:
//                if(isPast(autonTimer)){
//                    robot.setDrive();
//                }




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
