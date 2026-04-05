package org.firstinspires.ftc.teamcode.robots.bobby;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class Localizer implements Subsystem {
    public static double TICKS_PER_REV = 537.7;
    public static double WHEEL_RADIUS = 1.476;
    public static double CHAIN_RATIO = 20.0 / 15;

    private final DriveTrain drive;
    private final BNO055IMU imu;

    private int lastLF;
    private int lastRF;
    private int lastLB;
    private int lastRB;

    private double headingOffset;
    private double heading;
    private double x;
    private double y;

    public Localizer(HardwareMap hardwareMap, DriveTrain drive, double angleStartRadians) {
        this.drive = drive;
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;
        imu.initialize(params);

        resetHeading();
        headingOffset -= angleStartRadians;
        heading = getHeading();

        lastLF = drive.getLeftFrontTicks();
        lastRF = drive.getRightFrontTicks();
        lastLB = drive.getLeftBackTicks();
        lastRB = drive.getRightBackTicks();
    }

    @Override
    public void update(Canvas fieldOverlay) {
        updatePose();
    }

    public void updatePose() {
        int lf = drive.getLeftFrontTicks();
        int rf = drive.getRightFrontTicks();
        int lb = drive.getLeftBackTicks();
        int rb = drive.getRightBackTicks();

        int dLF = lf - lastLF;
        int dRF = rf - lastRF;
        int dLB = lb - lastLB;
        int dRB = rb - lastRB;

        lastLF = lf;
        lastRF = rf;
        lastLB = lb;
        lastRB = rb;

        double dlf = ticksToInches(dLF);
        double drf = ticksToInches(dRF);
        double dlb = ticksToInches(dLB);
        double drb = ticksToInches(dRB);

        heading = getHeading();
        double dxR = (dlf + drf + dlb + drb) / 4.0;
        double dyR = (-dlf + drf + dlb - drb) / 4.0;

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        x += dyR * cos + dxR * sin;
        y += dyR * sin - dxR * cos;
    }

    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    public double getHeading() {
        double h = getRawHeading() - headingOffset;
        while (h > Math.PI) h -= 2 * Math.PI;
        while (h < -Math.PI) h += 2 * Math.PI;
        return h;
    }

    public void resetHeading() {
        headingOffset = getRawHeading();
        heading = 0.0;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setPose(double x, double y) {
        this.x = x;
        this.y = y;
    }

    private double ticksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * CHAIN_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public void stop() {
    }

    @Override
    public void resetStates() {
        x = 0;
        y = 0;
        resetHeading();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("x (inches)", x);
        telemetry.put("y (inches)", y);
        telemetry.put("heading (rad)", heading);
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Localization";
    }
}