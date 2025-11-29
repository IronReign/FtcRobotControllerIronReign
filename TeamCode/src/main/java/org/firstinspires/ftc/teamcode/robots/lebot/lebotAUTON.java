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



        public static double FORWARD=1.21;
        public static double BUFFER=.3;
        public static double LONGBUFFER= 1.9;        //2.2
        public static double BACKWARD=.255;     //.255


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


        //forward vertical for one tile practice    vertical=4000;

        //angled towards first three balls
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
