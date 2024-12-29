//package org.firstinspires.ftc.teamcode.robots.giant;
//
//import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
//import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
//
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.robotcore.internal.system.Misc;
//
//import java.util.Map;
//
//@Autonomous(name = "GIANTAUTON")
//@Config(value = "GiantAuton")
//public class NewGiantAuton extends giantOpMode{
//
//    public static double FORWARD_SLAM=1.75;
//    public static double BUFFER=.2;
//    public static double BACKWARD=1.1;
//
//    Robot robot;
//    //HardwareMap hardwareMap;
//
//    @Override
//    public void init() {
//        robot = new Robot(hardwareMap, gamepad1);
//        robot.init();
//    }
//
//    @Override
//    public void loop() {
//        robot.update(new Canvas());
//        execute();
//        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
//    }
//
//    //forward one tile, turn 135, forward 1.25, extend arm, drop
//    int autonIndex = 0;
//    long autonTimer = 0;
//    public void execute() {
//        switch (autonIndex) {
//            case 0:
//                autonTimer = futureTime(BUFFER);
//                robot.grabBlock();
//                robot.setShoulderSpeed(1);
//                autonIndex++;
//                break;
//            case 1:
//                if(isPast(autonTimer)){
//                    autonTimer=futureTime(BUFFER);
//                    robot.setDrive(-.67, 0, 0);
//                    autonIndex++;
//                }
//                break;
//
//            case 2:
//                if(isPast(autonTimer)){
//                    autonTimer = futureTime(FORWARD_SLAM);
//                    robot.setDrive(0, 0, 0);
//                    robot.rotateBar();
//                    autonIndex++;
//                }
//                break;
//            case 3:
//                if(robot.getRotate()>1545){
//                    robot.unstickArm();
//                    autonIndex++;
//                }
//                break;
//            case 4:
//                if(robot.getExtend()>7845){
//                    robot.reachUp();
//                    autonIndex++;
//                }
//                break;
//            case 5:
//                if(robot.getExtend()<5145){
//                    autonTimer=futureTime(FORWARD_SLAM);
//                    robot.setDrive(-.5,0,0);
//                    autonIndex++;
//                }
//                break;
//            case 6:
//                if(isPast(autonTimer)){
//                    autonTimer=futureTime(BUFFER);
//                    robot.setDrive(0,0,0);
//                    autonIndex++;
//                }
//                break;
//            case 7:
//                if(isPast(autonTimer)){
//                    robot.setArmSpeed(.7);
//                    robot.pullDown();
//                    autonIndex++;
//                }
//                break;
//            case 8:
//                if(robot.getExtend()<1970){
//                    autonTimer=futureTime(BACKWARD);
//                    robot.dropBlock();
//                    robot.setDrive(.4,0,0);
//                    autonIndex++;
//                }
//                break;
//            case 9:
//                if(isPast(autonTimer)){
//                    robot.setDrive(0,0,0);
//                    robot.setArmSpeed(1);
//                    robot.resetE();
//                }
//                break;
//            case 10:
//                if(robot.getExtend()>2100){
//                    robot.resetR();
//                }
//
//        }
//    }
//
//    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
//        telemetry.addLine(telemetryName);
//
//        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
//            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
//            telemetry.addLine(line);
//            robot.getTelemetry(true);
//            //telemetry.addLine(""+autonIndex);
//        }
//
//        telemetry.addLine();
//    }
//}
