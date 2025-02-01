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
    public static double BUFFER=.23;
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

//    public void forward(double length, double direction){
//        if (!moving){
//            // Number of encoder ticks per distance
//            ticks = (int)((length/wheelCircum)*ticksrev);
//
//            // Assign initial encoder values
//            startpos = robot.vertical.getCurrentPosition();
//
//            // Indicate Vertical/Horizontal
//            vertical = true;
//            horizontal = false;
//
//            // Travel Distance
//            robot.mecanumDrive(-direction,0,0);
//
//            // Update moving
//            moving = true;
//        }
//    }


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
                robot.setUpExtend(2250);
                robot.resetDrive();
                autonIndex++;
                break;
            case 1:
                if(robot.getUpExtend()>2000){
                    robot.setDrive(.8,0,0);
                    autonIndex++;
                }

                break;
            case 2:
                if(robot.getVert()<=-650){
              //  if(robot.thereYetH(0) && robot.thereYetV(4000)){
                    autonTimer=futureTime(BUFFER);
                    robot.setDrive(0,0,0);
                    robot.resetDrive();
                    autonIndex++;
                }
                break;
            case 3:
                if(isPast(autonTimer)){
                    robot.setUpExtend(1950);        //2250
                    autonTimer=futureTime(BUFFER);
                    autonIndex++;
                }
                break;
            case 4:
                if(robot.getUpExtend()<1955){
                    robot.setDrive(-.3,0,0);
                    autonIndex++;

                }
                break;
            case 5:
                if(robot.getVert()>1800){
                    robot.setDrive(0,0,0);
                    autonTimer=futureTime(BUFFER);
                    autonIndex++;
                }
                break;
            case 6:
                if(isPast(autonTimer)){
                    robot.open();

                    autonIndex++;

                }
                break;
            case 7:
                robot.setDrive(0,.8,0);
                autonTimer=futureTime(3);
                autonIndex++;
                break;
            case 8:
                if(isPast(autonTimer)){
                    robot.setDrive(-.5,0,0);
                    autonTimer=futureTime(1);
                    autonIndex++;
                }
                break;
            case 9:
                if(isPast(autonTimer)){
                    robot.setDrive(0,0,0);
                }




//            case 0:
//                //autonTimer=futureTime(BUFFER);
//                robot.grabBlock();
//                autonIndex++;
//                break;
//            case 1:
//                if(robot.getClaw()>1095){
//                    robot.setRotate(1270);
//                    autonIndex++;
//                }
//                break;
//            case 2:
//                if(robot.getRotate()>1265){
//                    robot.setExtend(3668);
//                    autonIndex++;
//                }
//                break;
//            case 3:
//                if(robot.getExtend()>3665){
//                    autonTimer = futureTime(FORWARD);
//                    robot.setDrive(-.82,0,0);
//                    autonIndex++;
//                }
//                break;
//            case 4:
//                if(isPast(autonTimer)){
//                    autonTimer=futureTime(LONGBUFFER);
//                    robot.setDrive(0,0,0);
//                    autonIndex++;
//                }
//                break;
//            case 5:
//                if(isPast(autonTimer)){
//                    robot.setExtend(1160);      //working 1130
//                    autonIndex++;
//                }
//                break;
//            case 6:
//                if(robot.getExtend()<1165){     //working 1135
//                    autonTimer=futureTime(BACKWARD);
//                    robot.setDrive(.8,0,0);
//                    autonIndex++;
//                }
//                break;
//            case 7:
//                if(isPast(autonTimer)){
//                    autonTimer = futureTime(BUFFER);
//                    robot.setDrive(0,0,0);
//
//
//                    autonIndex++;
//                }
//                break;
//            case 8:
//                if(isPast(autonTimer)){
//                    robot.setClawP();
//                    autonIndex++;
//                }
//                break;
//            case 9:
//                if(robot.getClaw()>1600){
//                    robot.setExtend(2);
//                    autonIndex++;
//                }
//                break;
//            case 10:
//                if(robot.getExtend()<5){
//                    robot.setRotate(5);
//                    autonIndex++;
//                }
//                break;
//            case 11:
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
