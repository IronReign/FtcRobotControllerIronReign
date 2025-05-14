package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TriSwerve", group = "Challenge")
public class TriSwerveOp extends OpMode {

    TriSwerve robot;
    StickyGamepad stickyGamepad1;

    @Override
    public void init() {
        robot = new TriSwerve(hardwareMap);
        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        // Process the left joystick (x and y) to compute desired velocity.
        // hold left bumper to allow drive motor, otherwise only steering will update
        if (gamepad1.left_bumper)
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
        telemetry.addData("\nMODULE", "1");
        telemetry.addData("Module Target Angle", robot.swerveModule1.getTargetAngle());
        telemetry.addData("Module Current Angle", robot.swerveModule1.getCurrentAngle());
        telemetry.addData("Yaw Error", robot.swerveModule1.getYawError());
        telemetry.addData("Drive Speed", robot.swerveModule1.getDrivePowerActual());
        telemetry.addData("Drive Amps", robot.swerveModule1.getDriveAmps());

        telemetry.addData("\nMODULE", "2");
        telemetry.addData("Module Target Angle", robot.swerveModule2.getTargetAngle());
        telemetry.addData("Module Current Angle", robot.swerveModule2.getCurrentAngle());
        telemetry.addData("Yaw Error", robot.swerveModule2.getYawError());
        telemetry.addData("Drive Speed", robot.swerveModule2.getDrivePowerActual());
        telemetry.addData("Drive Amps", robot.swerveModule2.getDriveAmps());

        telemetry.addData("\nMODULE", "3");
        telemetry.addData("Module Target Angle", robot.swerveModule3.getTargetAngle());
        telemetry.addData("Module Current Angle", robot.swerveModule3.getCurrentAngle());
        telemetry.addData("Yaw Error", robot.swerveModule3.getYawError());
        telemetry.addData("Drive Speed", robot.swerveModule3.getDrivePowerActual());
        telemetry.addData("Drive Amps", robot.swerveModule3.getDriveAmps());
    }
}
