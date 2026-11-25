package org.firstinspires.ftc.teamcode.robots.Catapultabot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config(value = "CatapultAuton")
@Autonomous(name = "Catapultabot", group = "Catapultabot")
public class CatapultAuton extends OpMode {

    private Robot robot;

    private enum Action {
        LeaveDoc,
        Park,
        Wait,
    }

    private final ElapsedTime timer = new ElapsedTime();
    private static long leaveTime_ms = 1000L;

    private Action stage = Action.Wait;
    private boolean left = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        timer.reset();
        stage = Action.LeaveDoc;
    }

    @Override
    public void loop() {
        switch(stage){
            case LeaveDoc:

                if(!robot.drive_train.driving){
                    robot.drive_train.drive(0.1, 0.0, 0.0);
                    timer.reset();
                } else{
                    if(timer.milliseconds()>=leaveTime_ms){
                        robot.drive_train.drive(0, 0, 0);
                        left = true;
                        stage = Action.Park;
                    }
                }

                break;
            case Park:

                if(!robot.drive_train.driving){
                    robot.drive_train.drive(-0.1, 0.0, 0.0);
                    timer.reset();
                } else{
                    if(timer.milliseconds()>=leaveTime_ms){
                        robot.drive_train.drive(0, 0, 0);
                        stage = Action.Wait;
                    }
                }

                break;
            case Wait:
                break;
        }

        robot.readSensors();
        robot.calc(new Canvas());
        robot.act();

        telemetry.addData("Stage", stage);
        telemetry.update();

    }

    @Override
    public void stop(){robot.stop();}
}
