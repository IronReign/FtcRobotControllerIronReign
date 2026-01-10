package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;

@Disabled

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Swerve", group = "Challenge")
public class SwerveOp extends OpMode {

    MonoSwerve robot;
    StickyGamepad stickyGamepad1;

    @Override
    public void init() {
        robot = new MonoSwerve(hardwareMap);
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
        telemetry.addData("Module Target Angle", robot.swerveModule.getTargetAngle());
        telemetry.addData("Module Current Angle", robot.swerveModule.getCurrentAngle());
        telemetry.addData("Yaw Error", robot.swerveModule.getYawError());
        telemetry.addData("Drive Speed", robot.swerveModule.getDrivePowerActual());
        telemetry.addData("Drive Amps", robot.swerveModule.getDriveAmps());
    }
}
