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


@Autonomous(name = "auton")
@Config(value = "auton")
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
                    robot.setShoulder(1270);        //1350
                }
                if(robot.getShoulder()>1200){
                    autonIndex++;
                }

                break;
            case 2:
              //  robot.driveDistance(13,.5);
                if (robot.driveDistance(19.3,.8)){    //17.5    .5
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
                    autonIndex++;
                }
                break;
            case 5:
                robot.wallGrab();
                autonIndex++;
                break;
            case 6:
                autonIndex++;
                break;
//            case 6:
//                if(robot.getShoulder()<770){
//                    robot.setUpExtend(1300);
//                    autonIndex++;
//                 //   robot.setDrive();
//                }
//                break;
            case 7:
                if(robot.driveDistance(34,.8)){
                    autonIndex++;
                }
                break;
            case 8:
                if(robot.strafe(32,.8)){
                    robot.resetDrive();
                    autonTimer=futureTime(.7);
                    autonIndex++;
                }
                break;
            case 9:
                if(isPast(autonTimer)){
                    robot.setDrive(0,0,-.8);
                    robot.setOutPower(.7);
                    autonIndex++;
                }
                break;
            case 10:
                if(robot.getHor()>=2270 &&robot.getVert()>=6270){

                    robot.setDrive(0,0,0);    //930
                    robot.setTilt(950);
                    robot.setOutExtend(2430);
                    robot.suck();
                    autonIndex++;
                }
                break;
            case 11:
//                if(robot.strafe(38,.4)){
//
//                }
//                if(robot.strafe(34,.4)){
//
//                }
                if(!robot.getSuck()){
                    robot.setOutPower(1);
                    robot.resetDrive();
                    robot.setDrive(0,0,-.8);
                    autonIndex++;
                }
               // if(robot.getTilt()<980){

                  //  autonTimer=futureTime(BUFFER);

                //}
                break;

            case 12:
                    autonIndex++;

                break;
            case 13:
                if(robot.getHor()>=2270 &&robot.getVert()>=6270){           //robot.getHor()>=3550 &&robot.getVert()>=7550
                    robot.setDrive(0,0,0);
                    robot.setOutExtend(1400);
                    autonIndex++;
                }
                break;
            case 14:
                if(robot.getOutExtend()>1350){
                    autonTimer=futureTime(1);
                    robot.spit(true);
                    autonIndex++;
                }
                break;
            case 15:
                if(isPast(autonTimer)){
                    robot.spit(false);
                    robot.setOutExtend(100);
                    robot.setShoulder(1450);
                    robot.resetDrive();
                    robot.setDrive(0,0,-.8);
                    autonIndex++;
                }
                break;
            case 16:
                if(robot.getHor()>=2070 &&robot.getVert()>=6070){           //robot.getHor()>=3550 &&robot.getVert()>=7550
                    robot.setDrive(0,0,0);
                    autonIndex++;
                }
                break;

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
