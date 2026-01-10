package org.firstinspires.ftc.teamcode.robots.giant;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;


import java.util.Map;


@Disabled


@Autonomous(name = "auton SPECIMEN")
//@Config(value = "auton SPECIMEN")
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
        robot.autonMotors();
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
                robot.setUpExtend(2280);        //2285
                autonIndex++;
                break;
            case 1:
                if(robot.getUpMotor()>800){
                    robot.resetDrive();
                    robot.setShoulder(1250+420);        //1350
                }
                if(robot.getShoulder()>1230+420){
                    autonIndex++;
                }

                break;
            case 2:
              //  robot.driveDistance(13,.5);
                if (robot.driveDistance(13,1)){    //12.75    .5
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
                    //robot.wallGrab();
                    robot.setShoulder(1450+420);
                    robot.setUpExtend(8);
                    autonIndex++;
                }
                break;
            case 5:
                if(robot.driveDistance(50,1)){
//                    autonTimer=futureTime(.4);
//                    robot.setDrive(0,0,-1);
                    autonIndex++;
                }
                break;
            case 6:
                //if(robot.turnUntilDegreesIMU(-90, 1)) {
//                if (isPast(autonTimer)) {
//                    robot.setDrive(0,0,0)
//                    ;
//                    //autonTimer=futureTime(.5);
                    autonIndex++;
//                }
//                    if (robot.turnUntilDegreesIMU(-90, 1)) {
//                        autonIndex++;
//                    }

                break;

//                if(isPast(autonTimer)){
//                    robot.resetDrive();
//                    //robot.setDrive(0,1,0);
//                    autonIndex++;
//                }
//                break;
            case 7:
                if(robot.turnUntilDegreesIMU(-90, .6)) {
                    autonIndex++;
                }
                break;
                //if(isPast(autonTimer)) {

                //}
//                if(robot.strafe(43,1)){       //speed up??
//                    autonIndex++;
//                }
            case 8:
                if (robot.driveDistance(50, 1)) {
//                    autonTimer=futureTime(.4);
//                    robot.setDrive(0,0,-1);
//                        robot.setTilt(760);
//                        robot.setOutExtend(750);
//                        robot.suck();
//                        autonTimer = futureTime(.5);

                    autonIndex++;//david is fat
                }

//                if(robot.turnUntilDegreesIMU(-180,1)){      //robot.getHor()<=(-3780) &&robot.getVert()<=(-7180)
//                //if(isPast(autonTimer)){
//                    autonTimer=futureTime(.4);
//                    robot.setDrive(-.1,-.5,0);
//                    robot.resetDrive();
//
//                   // robot.setDrive(0,0,-.5);    //930
//                    //autonTimer=futureTime(.01);
//                     //autonTimer=futureTime(1.1);
//                    //autonTimer=futureTime(1.5);
//                    autonIndex++;
//                }
                break;
            case 9:
                autonIndex++;
//                if(isPast(autonTimer)){
//                    robot.setDrive(0,0,0);
//                    autonIndex++;
//                }
                //if(isPast(autonTimer)){
//                    robot.setDrive(0,-.5,0);
//                    autonTimer=futureTime(.5);
                //}
                break;
            case 10:
                if(robot.turnUntilDegreesIMU(-180,.6)) {      //robot.getHor()<=(-3780) &&robot.getVert()<=(-7180)
                    autonIndex++;
                }
// if(isPast(autonTimer)){
//                    autonTimer=futureTime(.4);
//                    robot.setDrive(-.1,-.5,0);
//                    robot.resetDrive();
//
//                   // robot.setDrive(0,0,-.5);    //930
//                    //autonTimer=futureTime(.01);
//                     //autonTimer=futureTime(1.1);
//                    //autonTimer=futureTime(1.5);
//                    autonIndex++;
//                if(isPast(autonTimer)){
//                    robot.setDrive(0,0,0);
//                //if(!robot.getSuck()){
//                    robot.setTilt(830);
//                    robot.setOutExtend(220);
//                }
                break;
            case 11:
//                if(robot.turnUntilDegreesIMU(0,1)){
//                    autonTimer=futureTime(.23);
//                    robot.spit(true);
                    autonIndex++;
//                }
                break;
            case 12:
//                if(isPast(autonTimer)){
//                    robot.spit(false);
//                    robot.setOutExtend(0);
                    autonIndex++;
//                }
                break;
            case 13:
                //if(robot.turnUntilDegreesIMU(-180,1)){
                    autonIndex++;
                //}
                break;
//            case 13:
//                //if(isPast(autonTimer)){
//                    autonIndex++;
//                //}
//                break;
            case 14:
                if(robot.driveDistance(18.35,1)){
                    autonTimer=futureTime(.1);
                    autonIndex++;

                }
                break;
            case 15:
                if(isPast(autonTimer)){
                    robot.close();
                    autonTimer=futureTime(.05);
                    autonIndex++;
                }
                break;
            case 16:
                if(isPast(autonTimer)){
                    robot.hookit();
                    robot.setUpExtend(1500);
                    autonIndex++;
                }
                break;
            case 17:
                if(robot.getUpExtend()>200){
                    autonIndex++;


                }
                break;
            case 18:        //47.8
                if(robot.driveDistance(45,1)){        //47.8
//                    autonTimer=futureTime(.4);
//                    robot.setDrive(0,0,1);
                    robot.setShoulder(1250+420);
                    autonIndex++;
//                if(isPast(autonTimer))
                }
                break;
            case 19:
                if(robot.turnUntilDegreesIMU(-90,.6)){
                    robot.setUpExtend(2302);        //2290
                    robot.resetDrive();
                    autonTimer = futureTime(1);
                    robot.setDrive(-1, 0, 0);
                //if(isPast(autonTimer)){
//                    robot.setDrive(0,0,0);
//                    autonTimer=futureTime(.05);
                    autonIndex++;
                }
                break;
            case 20:
                //if(robot.turnUntilDegreesIMU(-90,1)) {
                    //if (isPast(autonTimer)) {

                        autonIndex++;
                    //}


//                if(robot.driveDistance(70,1)){   //robot.getHor()<-13000
//                    robot.setUpExtend(2302);        //2290
//                    robot.resetDrive();
//                    autonTimer=futureTime(1.1);
//                    robot.setDrive(-1,0,0);
//                    autonIndex++;
//                }
                break;
            case 21:
                if(isPast(autonTimer)){
                    robot.setDrive(0,0,0);
                    autonIndex++;
                }
                break;
            case 22:
                if(robot.turnUntilDegreesIMU(0,.6)){
                    autonIndex++;
                }
                break;
            case 23:
                if (robot.driveDistance(13.2,1)){    //11.8   .5
                    autonTimer=futureTime(.1);
                    autonIndex++;
                }
                break;
            case 24:
                if(isPast(autonTimer)){
                    //robot.downHook();
                    robot.setUpExtend(1310);
                    autonTimer=futureTime(.6);
                    autonIndex++;
                }
                break;
            case 25:
                if(isPast(autonTimer)){
                    robot.open();
                    robot.wallGrab();
                    autonIndex++;
                }
                break;
            case 26:
                if(robot.driveDistance(35,1)){
                    autonTimer=futureTime(.5);
                    robot.setUpExtend(1600);
                    autonIndex++;
                }
                break;
            case 27:
                if(isPast(autonTimer)) {
                    robot.setShoulder(750+420);
                    autonTimer=futureTime(.2);
                    autonIndex++;
                }
                break;
            case 28:
                if(isPast(autonTimer)){
                    robot.setUpExtend(0);
                    autonIndex++;
                }
                break;
            case 29:
                if(robot.strafe(40,1)){           //speedup???
                    autonTimer=futureTime(.44);
                    robot.setDrive(-1,0,0);
                    autonIndex++;
                }
                break;
            case 30:
                if(isPast(autonTimer)){
                    robot.setDrive(0,0,0);
                    autonIndex++;
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
