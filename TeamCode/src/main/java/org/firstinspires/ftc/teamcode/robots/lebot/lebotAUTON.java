package org.firstinspires.ftc.teamcode.robots.lebot;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;

import java.util.Map;
@Autonomous(name = "auton lebot")
@Config(value = "auton lebot")
public class lebotAUTON extends OpMode{
//
//
//
//        public static double FORWARD=1.21;
//        public static double BUFFER=.3;
//        public static double LONGBUFFER= 1.9;        //2.2
//        public static double BACKWARD=.255;     //.255
//
//
        public double referenceAngle=35;
        Robot robot;
        StickyGamepad g1=null;
        //HardwareMap hardwareMap;

    //start facing goal towards tag, move back till distance sensor sense 80.4 (80)

        @Override
        public void init() {
            robot = new Robot(hardwareMap, gamepad1);
            robot.init();
            robot.setPaddleDown();
            robot.setMinShootSpeed(845);
            //  robot.setRotate(950);
        }
        @Override
        public void init_loop(){
            g1=new StickyGamepad(gamepad1);
            g1.update();
            robot.setPaddleDown();
            if(g1.b){
                robot.setRedALliance(true);
                robot.switchPipeline(1);
                robot.setAutoAngle(-Math.abs(referenceAngle));
            }
            if(g1.x){
                robot.setRedALliance(false);
                robot.switchPipeline(0);
                robot.setAutoAngle(Math.abs(referenceAngle));
            }
            robot.update(new Canvas());
            handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());

        }


        @Override
        public void loop() {
            robot.update(new Canvas());
            robot.updateDistance();
            execute();
            handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
        }
//
//
//        //forward vertical for one tile practice    vertical=4000;
//
//    /*
//    - robot angled towards the three balls
//    - robot aligned for intake?
//        - timer
//        - fireBall()
//     */
        int autonIndex = 0;
        long autonTimer = 0;
        public void execute() {
            switch (autonIndex) {
                case 0:
                    if(robot.getRedALliance()){
                        robot.setAutoAngle(-Math.abs(referenceAngle));
                    }else{
                        robot.setAutoAngle(Math.abs(referenceAngle));
                    }
                    robot.setDrivetrain(-.5, 0);       //2285
                    //autonTimer = futureTime(.2);
                    autonIndex++;
                    break;
                case 1:
                    if (robot.getDistFromTag()>75) {
                        robot.setDrivetrain(0, 0);
                        autonIndex++;
                    }
                    break;
                case 2:
                    if(robot.tx()){
                        autonTimer=futureTime(.5);
                        robot.setTurningT(true);
                        autonIndex++;
                    }
                    break;

                case 3:
                    if(isPast(autonTimer)){
                        robot.setDrivetrain(0,0);
                    //if(!robot.getTurningT()){
                        robot.setMinShootSpeed(robot.getShootingSpeed());
                        autonIndex++;
                    }
                    break;
                case 4:
                    //if(robot.getMinShooterSpeed()>500 && robot.getMinShooterSpeed()<1100){
                        robot.resetShootIndex();
                        robot.setShoot(true);
                        autonIndex++;
                    //}
                case 5:
                    if(!robot.getShoot()){
                        autonTimer=futureTime(.5);
                        autonIndex++;
                    }
                    break;
                case 6:
                    if(isPast(autonTimer)){
                        autonTimer=futureTime(.05);
                        robot.intakeOn();
                        autonIndex++;
                    }
                    break;
                case 7:
                    if(isPast(autonTimer)){
                        robot.intakeOff();
                        robot.resetShootIndex();
                        robot.setShoot(true);
                        autonIndex++;
                    }
                    break;
                case 8:
                    if(!robot.getShoot()){
                        autonTimer=futureTime(.5);
                        autonIndex++;
                    }
                    break;
                case 9:
                    if(isPast(autonTimer)){
                        autonTimer=futureTime(.05);
                        robot.intakeOn();
                        autonIndex++;
                    }
                    break;
                case 10:
                    if(isPast(autonTimer)){
                        robot.intakeOff();
                        robot.resetShootIndex();
                        robot.setShoot(true);
                        autonIndex++;
                    }
                    break;
                case 11:
                    if(!robot.getShoot()){
                        autonTimer=futureTime(.8);
                        robot.setShoot(false);
                        robot.shoot(false);
                        robot.setTurningAuto(true);
                        //autonTimer=futureTime(1.2);
                        autonIndex++;
                    }
                    break;
                case 12:
                    if(isPast(autonTimer)){
                        autonTimer=futureTime(.4);
                        robot.setDrivetrain(1,0);
                        autonIndex++;
                    }
                    break;
                case 13:
                    if(isPast(autonTimer)){
                        robot.setDrivetrain(0,0);
                        autonIndex++;
                    }
                    break;

            }
        }
//
//        public void execute2(){
//            switch(autonIndex){
//                case 0:
//                    robot.setDrivetrain(1, 0);
//                    autonIndex++;
//                    break;
//                case 1:
//                    if(robot.getFrontDistance()<2) {
//                        autonIndex++;
//                        robot.setDrivetrain(0,0);
//                    }
//                    break;
//                case 2:
//                    robot.setDrivetrain(0, 1);
//                    autonTimer = futureTime(.2);
//                    autonIndex++;
//                    break;
//                case 3:
//                    if(isPast(autonTimer)){
//                        autonIndex++;
//                        robot.setDrivetrain(0,0);
//                    }
//                    break;
//                case 4:
//                    //TODO: Distance adjusting function equivalent -- still have to finish
//                    double target = robot.calculateDist();
//                    double current = robot.getFrontDistance();
//                    double error = current - target;
//                    autonTimer = futureTime(2);
//
//                    if (Math.abs(error) > 0.02) {
//                        if (error > 0) {
//                            robot.setDrivetrain(1, 0);
//                        } else {
//                            robot.setDrivetrain(-1, 0);
//                        }
//                    }
//                    autonIndex++;
//                    break;
//                case 5:
//                    //NOTE: Depending on alliance, robot has to turn to range somewhat near goal before turnItShoot can run
//                case 6:
//                    robot.fireBall();
//                    break;
//            }
//        }
//

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
