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


@Autonomous(name = "auton TEST")
@Config(value = "auton TEST")
public class TestAuton extends OpMode {


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
                if (robot.driveDistance(12.95,1)){    //12.75    .5
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
                    autonTimer=futureTime(.5);
                    robot.setDrive(-1,0,0);
                    autonIndex++;
                }
                break;
            case 5:
                if(isPast(autonTimer)){
                    robot.setDrive(0,0,0);
                //if(robot.driveDistance(35,1)){
                    autonTimer=futureTime(.5);
                    robot.setUpExtend(1600);
                    autonIndex++;
                }
                break;
            case 6:
                if(isPast(autonTimer)) {
                    robot.setShoulder(750+420);
                    autonTimer=futureTime(.2);
                    autonIndex++;
                }
                break;
            case 7:
                if(isPast(autonTimer)){
                    robot.setUpExtend(0);
                    autonIndex++;
                }
                break;
//            case 8:
//                if(robot.strafe(40,1)){           //speedup???
//                    autonTimer=futureTime(.48);
//                    robot.setDrive(-1,0,0);
//                    autonIndex++;
//                }
//                break;
//            case 9:
//                if(isPast(autonTimer)){
//                    robot.setDrive(0,0,0);
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
