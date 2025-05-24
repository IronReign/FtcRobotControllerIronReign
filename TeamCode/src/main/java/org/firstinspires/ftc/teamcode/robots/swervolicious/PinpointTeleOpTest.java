package org.firstinspires.ftc.teamcode.robots.swervolicious;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.swervolicious.rr_localize.PinpointLocalizer;

/**
 * Simple TeleOp to validate Pinpoint odometry directions and scaling on the swerve chassis.
 * <p>
 * Controls:
 * <ul>
 *     <li>Left stick – translate (field‑centric)</li>
 *     <li>Right stick X – rotate</li>
 *     <li>B button – toggle alliance orientation (red ↔ blue)</li>
 * </ul>
 */
@TeleOp(name = "Pinpoint Test (Swerve)", group = "Testing")
public class PinpointTeleOpTest extends OpMode {

    private DriveTrain driveTrain;
    private boolean isRedAlliance = true;
    private boolean lastBState = false;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // NOTE: Robot reference is unused for basic driving/telemetry – pass null.
        driveTrain = new DriveTrain(hardwareMap, /* Robot */ null, /* simulated */ false);
        telemetry.addLine("Pinpoint TeleOp ready!");
    }

    @Override
    public void loop() {
        // Alliance‑toggle on gamepad1 B
        if (gamepad1.b && !lastBState) {
            isRedAlliance = !isRedAlliance;
        }
        lastBState = gamepad1.b;

        // Read gamepad: left stick XY translate, right stick X rotate
        double x = gamepad1.left_stick_x;   // strafe (right +)
        double y = -gamepad1.left_stick_y;  // forward + (negate to match joystick)
        double heading = -gamepad1.right_stick_x; // CCW + (negate for natural feel)

        driveTrain.fieldOrientedDrive(x, y, heading, isRedAlliance);
        driveTrain.update(null); // no dashboard Canvas in TeleOp

        // ---- Telemetry ----
        PinpointLocalizer pll = (PinpointLocalizer) driveTrain.localizer;
        telemetry.addData("Runtime", "%.1fs", runtime.seconds());
        telemetry.addLine("--- Pose (RR) ---");
        telemetry.addData("X (in)", "%.2f", pll.getPose().position.x);
        telemetry.addData("Y (in)", "%.2f", pll.getPose().position.y);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pll.getPose().heading.log()));

        telemetry.addLine("--- Pinpoint Raw ---");
        telemetry.addData("X ticks", pll.driver.getPosX());
        telemetry.addData("Y ticks", pll.driver.getPosY());
        telemetry.addData("Heading rad", "%.3f", pll.driver.getHeading());
        telemetry.addData("parDir", pll.initialParDirection);
        telemetry.addData("perpDir", pll.initialPerpDirection);

        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void stop() {
        driveTrain.stop();
    }
}
