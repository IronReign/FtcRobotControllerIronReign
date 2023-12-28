package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.Robot;
import org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems.Toggle;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

@Config("BoboGameVariables")
@TeleOp(name="BoboOpMode", group="Challenge")
public class BoboOp extends OpMode {
    public static Robot bobot;
    IMU imu;
    StickyGamepad stickyGamepad;
    Toggle toggle;
    Orientation angles;
    Acceleration gravity;
    MultipleTelemetry dashTelemetry;
    FtcDashboard dashboard;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        bobot = new Robot(dashTelemetry, hardwareMap);
        toggle = new Toggle(gamepad1);
        imu = new IMU(dashTelemetry, hardwareMap);
        dashTelemetry.setMsTransmissionInterval(25);
    }

    @Override
    public void loop() {
        toggle.gamepadUpdate();
        toggle.toggleSpeedMode();
        bobot.opDrive.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        bobot.droneLaunch.droneRelease(gamepad1.y);
        bobot.claw.clawArmLift(gamepad1.a);
        bobot.claw.clawArmLower(gamepad1.b);
        bobot.claw.armWristIn(gamepad1.dpad_down);
        bobot.claw.armWristOut(gamepad1.dpad_up);
        bobot.claw.openClaw(gamepad1.right_bumper);
        bobot.claw.closeClaw(gamepad1.left_bumper);
        bobot.opDrive.telemetryOutput();
        bobot.claw.telemetryOutput();
        bobot.droneLaunch.telemetryOutput();
        imu.telemetryOutput();
        dashTelemetry.update();
    }
}
