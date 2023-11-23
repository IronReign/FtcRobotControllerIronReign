package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config("BoboGameVariables")
@TeleOp(name="BoboOpMode", group="Challenge")
public class BoboBot extends OpMode {
    Robot bobo;
    @Override
    public void init() {
        bobo = new Robot(telemetry, hardwareMap);
    }

    @Override
    public void loop() {
        bobo.driveTrain.mechanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        bobo.droneLaunch.droneRelease(gamepad1.y);
        bobo.claw.clawArmLift(gamepad1.a);
        bobo.claw.clawArmLower(gamepad1.b);
        bobo.claw.armWristDown(gamepad1.dpad_down);
        bobo.claw.armWristUp(gamepad1.dpad_up);
        bobo.claw.openClaw(gamepad1.right_bumper);
        bobo.claw.closeClaw(gamepad1.left_bumper);
        bobo.driveTrain.telemetryOutput();
        bobo.claw.telemetryOutput();
        bobo.droneLaunch.telemetryOutput();
    }
}
