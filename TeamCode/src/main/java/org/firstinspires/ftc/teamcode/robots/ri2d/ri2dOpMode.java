package org.firstinspires.ftc.teamcode.robots.ri2d;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ri2dOpMode extends OpMode {


    public static ri2dbot robot;
    public static DCRI2D dc;

    @Override
    public void init() {
        robot = new ri2dbot(hardwareMap);
        dc = new DCRI2D(gamepad1);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void loop() {
        dc.joystickDrive();
        updateTelemetry();
    }

    public void updateTelemetry() {


//      **** DIRECT DRIVE ONLY ****
//        telemetry.addData("dampen?: ", dampenRotation);
//        telemetry.addData("goPower: ", "stick (%.2f), power (%.2f)", gamepad1.left_stick_y, robot.goPower);
    }
}
