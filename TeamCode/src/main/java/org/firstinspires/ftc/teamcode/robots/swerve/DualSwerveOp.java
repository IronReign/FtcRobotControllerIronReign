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
        double leftDeflection = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double rightDeflection = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
//        to toggle drive and direction separately
//        if (gamepad1.left_bumper)
        if(stickyGamepad1.a){
            DualSwerve.enabled = !DualSwerve.enabled;
        }
        if(stickyGamepad1.b)
        {
            robot.resetEncoders();
        }
        if (leftDeflection > 0.2)
            robot.processDriverInputDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, true);
        else
            robot.processDriverInputDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, false);

        if(rightDeflection > 0.2)
            robot.processDriverInputRotation(gamepad1.right_stick_x, -gamepad1.right_stick_y, true);
        else
            robot.processDriverInputRotation(gamepad1.right_stick_x, -gamepad1.right_stick_y, false);

        // Update chassis (which in turn updates the swerve module and IMU data)
        robot.update(new Canvas());
        updateTelemetry();
    }

    public void updateTelemetry() {
        // Telemetry now shows chassis heading and swerve module status.
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

        telemetry.addData("actual chassis angle", robot.chassisHeading);
        telemetry.addData("target chassis angle", DualSwerve.desiredChassisAngle);
        telemetry.addData("pid correction", DualSwerve.PIDCorrection);
        telemetry.addData("pid error", DualSwerve.PIDError);
    }
}
