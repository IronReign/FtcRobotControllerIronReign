package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Robot;

@Config("BoboGameVariables")
@TeleOp(name="BoboOpMode", group="Challenge")
public class BoboOp extends OpMode {
    Robot bobot;
    IMU imu;

    Orientation angles;
    Acceleration gravity;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        bobot = new Robot(dashTelemetry, hardwareMap);
        imu = new IMU(dashTelemetry, hardwareMap);
        //imu.initialize(parameters);
        dashTelemetry.setMsTransmissionInterval(25);
    }

    @Override
    public void loop() {
        bobot.opDrive.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        bobot.droneLaunch.droneRelease(gamepad1.y);
        bobot.claw.clawArmLift(gamepad1.a);
        bobot.claw.clawArmLower(gamepad1.b);
        bobot.claw.armWristIn(gamepad1.dpad_down);
        bobot.claw.armWristOut(gamepad1.dpad_up);
        bobot.claw.openClaw(gamepad1.right_bumper);
        bobot.claw.closeClaw(gamepad1.left_bumper);
        bobot.claw.inTake(gamepad1.x);
        bobot.opDrive.telemetryOutput();
        bobot.claw.telemetryOutput();
        bobot.droneLaunch.telemetryOutput();
        imu.telemetryOutput();
        dashTelemetry.update();
    }
}
