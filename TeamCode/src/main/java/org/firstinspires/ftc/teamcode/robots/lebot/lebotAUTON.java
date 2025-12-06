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
        Robot robot;
        StickyGamepad g1=null;
        //HardwareMap hardwareMap;


        @Override
        public void init() {
            robot = new Robot(hardwareMap, gamepad1);
            robot.init(hardwareMap);
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
            }
            if(g1.x){
                robot.setRedALliance(false);
                robot.switchPipeline(0);
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
                    robot.setDrivetrain(1, 0);       //2285
                    autonTimer = futureTime(.2);
                    autonIndex++;
                    break;
                case 1:
                    if (isPast(autonTimer)) {
                        robot.setDrivetrain(0, 0);
                    }
                    break;
                case 2:

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
