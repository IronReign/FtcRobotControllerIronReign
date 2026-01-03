package org.firstinspires.ftc.teamcode.robots.swervolicious.rr_localize;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.rrQuickStart.Localizer;

import java.util.Objects;

/**
 * Pinpoint + RoadRunner localizer tuned for goBILDA 4‑Bar odometry pods.
 * <p>
 * The default offsets below are calculated from the measured pod locations
 * on the swerve chassis:
 * <ul>
 *     <li>Parallel (X) pod is 96.878 mm <b>left</b> of the robot center → +Y offset</li>
 *     <li>Perpendicular (Y) pod is 106.105 mm <b>forward</b> of the robot center → +X offset</li>
 * </ul>
 * Using the published scale of {@code 19.89436789 ticks/mm} for the goBILDA 4‑Bar pod
 * yields the tick constants below.
 */
@Config
public final class PinpointLocalizer implements Localizer {

    public static class Params {
        // (19.89436789 ticks/mm) × 96.878 mm ≈ -1927.3266 ticks
        public double parYTicks  = -1927.3266;   // right of center → negative Y
        // (19.89436789 ticks/mm) × 106.105 mm ≈ 2110.8919 ticks
        public double perpXTicks = 2110.89190497;
    }

    // Encoder scales (ticks per millimetre) for reference.
    private static final float goBILDA_SWINGARM_POD = 13.26291192f;
    private static final float goBILDA_4_BAR_POD    = 19.89436789f;

    public static org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.PinpointLocalizer.Params PARAMS = new org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.PinpointLocalizer.Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
