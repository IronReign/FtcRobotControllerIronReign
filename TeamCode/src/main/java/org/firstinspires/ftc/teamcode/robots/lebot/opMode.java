package org.firstinspires.ftc.teamcode.robots.lebot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "test")
public class opMode extends OpMode {

    tankDrive drivet = new tankDrive();

    double throttle, spin;

    @Override
    public void init() {
        drivet.init(hardwareMap);
    }

    @Override
    public void loop() {
        throttle = -gamepad1.left_stick_y;
        spin = -gamepad1.left_stick_x;

        drivet.drive(throttle, spin);
    }
}
