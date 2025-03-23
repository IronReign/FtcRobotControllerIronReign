package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TwoSwerve", group = "Challenge")
public class DualSwerveOp extends OpMode {

    DualSwerve robot;
    StickyGamepad stickyGamepad1;

    @Override
    public void init() {
        robot = new DualSwerve(hardwareMap);
        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        // Process the left joystick (x and y) to compute desired velocity.
        // hold left bumper to allow drive motor, otherwise only steering will update
        double deflection = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//        to toggle drive and direction separately
//        if (gamepad1.left_bumper)
        if (deflection > 0.2)
            robot.processDriverInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, true);
        else
            robot.processDriverInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, false);

        // Update chassis (which in turn updates the swerve module and IMU data)
        robot.update(new Canvas());
        updateTelemetry();
    }

    public void updateTelemetry() {
        // Telemetry now shows chassis heading and swerve module status.
        telemetry.addData("Chassis Heading", robot.chassisHeading);
        telemetry.addData("Module Target Angle", robot.swerveModuleOne.getTargetAngle());
        telemetry.addData("Module Current Angle", robot.swerveModuleOne.getCurrentAngle());
        telemetry.addData("Yaw Error", robot.swerveModuleOne.getYawError());
        telemetry.addData("Drive Speed", robot.swerveModuleOne.getDrivePowerActual());
        telemetry.addData("Drive Amps", robot.swerveModuleOne.getDriveAmps());
        telemetry.addData("Desired Angle", robot.swerveModuleOne.getDesiredAngle());

        telemetry.addData("Module Target Angle", robot.swerveModuleTwo.getTargetAngle());
        telemetry.addData("Module Current Angle", robot.swerveModuleTwo.getCurrentAngle());
        telemetry.addData("Yaw Error", robot.swerveModuleTwo.getYawError());
        telemetry.addData("Drive Speed", robot.swerveModuleTwo.getDrivePowerActual());
        telemetry.addData("Drive Amps", robot.swerveModuleTwo.getDriveAmps());
        telemetry.addData("Desired Angle", robot.swerveModuleTwo.getDesiredAngle());
    }
}
