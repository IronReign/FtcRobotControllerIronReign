package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.SwerveModule;

//@Config
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TriSwerveOp", group = "Challenge")
public class TriSwerveOp extends OpMode {
    public static boolean quickTripEnabled = false;

    TriSwerve robot;
    StickyGamepad stickyGamepad1;

    @Override
    public void init() {
        robot = new TriSwerve(hardwareMap);
        stickyGamepad1 = new StickyGamepad(gamepad1);

    }

    public void init_loop() {
        stickyGamepad1.update();
        updateTelemetry();
        if (stickyGamepad1.b) {
            quickTripEnabled = !quickTripEnabled;
        }
        robot.calibrate(); // calibrate the serve modules - harmless if calibration is complete
    }
    public void start() {
        robot.gripperReady  =true;
        robot.limelight.start();
    }

    @Override
    public void loop() {
        if (quickTripEnabled) {
            if (robot.quickTrip())
                quickTripEnabled = false;
        } else {
            if (gamepad1.left_bumper)
                robot.processDriverInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, true);
            else {
                if (Math.abs(gamepad1.right_stick_x) > .2)
                    robot.rotate(-gamepad1.right_stick_x);
                else
                    robot.rotate(0);

                //robot.processDriverInput(gamepad1.left_stick_x, -gamepad1.left_stick_y, false);
            }


        }

        if(gamepad1.a) {
            robot.gripperOpen = !robot.gripperOpen;
        }
        // Process the left joystick (x and y) to compute desired velocity.
        // hold left bumper to allow drive motor, otherwise only steering will update


        // Update chassis (which in turn updates the swerve module and IMU data)
        robot.update(new Canvas());
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("gripper", robot.gripperOpen);
        telemetry.addData("quicktrip?", quickTripEnabled);
        telemetry.addData("getCanTx", robot.getCanTx());
        telemetry.addData("pose x", robot.localizer.getPose().position.x);
        telemetry.addData("pose y", robot.localizer.getPose().position.y);
        telemetry.addData("heading", robot.localizer.getPose().heading.log());
        // Telemetry now shows chassis heading and swerve module status.
        telemetry.addData("Chassis Heading", robot.chassisHeading);

        for(int i = 0; i < robot.modules.length; i++) {
            SwerveModule module = robot.modules[i];
            telemetry.addData("\nMODULE", i+1);
            telemetry.addData("Module Target Angle", module.getTargetAngle());
            telemetry.addData("Module Current Angle", module.getCurrentAngle());
            telemetry.addData("Yaw Power",module.getYawPower());
            telemetry.addData("Yaw Error", module.getYawError());
            telemetry.addData("Yaw Analog", module.getYawAnalog());
            telemetry.addData("atJump?", module.atJump());
            telemetry.addData("arr", module.pastAnalogValues);
            telemetry.addData("Drive Speed",module.getDrivePowerActual());
            telemetry.addData("Drive Amps", module.getDriveAmps());
        }
    }
}
