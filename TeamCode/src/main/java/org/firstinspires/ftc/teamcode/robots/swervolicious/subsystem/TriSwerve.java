package org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;
import org.firstinspires.ftc.teamcode.robots.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Map;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/**
 * Three-module IsoSwerve chassis (isosceles-triangle layout).
 *
 * +X - forward, +Y - left  (ROS/FRC convention).
 *
 * Left-stick X/Y → field-oriented translation (vx,vy)<br>
 * Right-stick X  → yaw-rate command ω  (left = CCW = +)
 */
//@Config(value = "IsoSwerve")
public class TriSwerve implements Subsystem {

    /* ---------------- constants & config ---------------- */

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(.045, 0, 0.5);
    public static double ticksPerDegree = 4920.0 / 360.0;
    public static double yawThreshold   = 10;                     // deg
    public static double rotateScale    = 1.0;                    // tune: ω = rightX * rotateScale
    public static double deadband       = 0.15;                   // joystick deadband
    public static double gyroOffset     = 0.0;                    // field zero = when robot starts

    /* ---------------- hardware ---------------- */

    private final SwerveModule back, left, right;
    private final BNO055IMU imu;
    private Orientation imuAngles = new Orientation();

    /* ---------------- geometry ---------------- */

    /** Module centres in mm, +X forward, +Y left */
    private final List<Vec2> pos = Arrays.asList(
            new Vec2(-191.029,   0.000),  // back
            new Vec2( 106.754,  158.500), // left  (was right-handed, sign fixed)
            new Vec2( 106.754, -158.500)  // right
    );

    private final Vec2 center;   // circum-centre of the three points
    private final double maxRadius; // furthest wheel distance from centre (mm)

    /* ---------------- runtime state ---------------- */

    private double chassisHeadingRad;  // radians (field frame)

    /* ---------------- constructor ---------------- */

    public TriSwerve(HardwareMap hw) {
        /* ---------- module creation ---------- */
        back  = makeModule(hw, "go0","yaw0","encoder0", "a0");
        left  = makeModule(hw, "go1","yaw1","encoder1", "a1");
        right = makeModule(hw, "go2","yaw2","encoder2", "a2");

        /* ---------- IMU ---------- */
        imu = hw.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.mode = BNO055IMU.SensorMode.IMU;
        p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(p);

        /* ---------- geometry pre-calc ---------- */
        center     = circumCenter(pos.get(0), pos.get(1), pos.get(2));
        maxRadius  = Math.max(
                Math.max(center.dist(pos.get(0)), center.dist(pos.get(1))),
                center.dist(pos.get(2)));

        // shift all wheel positions so centre becomes the origin:
        for (int i = 0; i < pos.size(); i++) {
            pos.set(i, pos.get(i).minus(center));
        }
    }

    private SwerveModule makeModule(HardwareMap hw,
                                    String driveName,
                                    String yawName,
                                    String encName,
                                    String analogName) {

        DcMotorEx drive = hw.get(DcMotorEx.class, driveName);
        CRServo   servo = hw.get(CRServo.class,    yawName);
        DcMotorEx enc   = hw.get(DcMotorEx.class,  encName);
        AnalogInput a = hw.get(AnalogInput.class, analogName);

        enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setMotorEnable();

        PIDController pid = new PIDController(pidCoefficients);
        pid.setOutputRange(-0.5, 0.5);
        pid.setContinuous();
        pid.setTolerance(50);
        pid.setInputRange(0, 360);
        pid.enable();

        return new SwerveModule(drive, servo, enc, a, pid, ticksPerDegree, yawThreshold);
    }


    /* ---------------- robot centric drive kinematics  ---------------- */
    private void driveRobotFrame(double vx, double vy, double omega) {

        double[] speeds  = new double[3];
        double[] anglesD = new double[3];

        Vec2[] wheels = {pos.get(0), pos.get(1), pos.get(2)};
        for (int i = 0; i < 3; i++) {
            double wheelVx = vx - omega * (wheels[i].y / maxRadius);
            double wheelVy = vy + omega * (wheels[i].x / maxRadius);

            speeds[i]  = Math.hypot(wheelVx, wheelVy);
            anglesD[i] = Math.toDegrees(Math.atan2(wheelVy, wheelVx));
        }

        double max = Math.max(Math.max(speeds[0], speeds[1]), speeds[2]);
        if (max > 1.0) for (int i = 0; i < speeds.length; i++) speeds[i] /= max;

        back .setDesiredState(anglesD[0], speeds[0]);
        left .setDesiredState(anglesD[1], speeds[1]);
        right.setDesiredState(anglesD[2], speeds[2]);
    }

    /* ---------------- driver robot centric interface ---------------- */

    public void drive(double lx, double ly, double rx) {
        lx = (Math.abs(lx) > deadband) ? lx : 0.0;
        ly = (Math.abs(ly) > deadband) ? ly : 0.0;
        rx = (Math.abs(rx) > deadband) ? rx : 0.0;

        double headingDeg = Math.toDegrees(chassisHeadingRad);
        double[] rot = rotateFieldToRobot(lx, ly, headingDeg);
        double vx = rot[0];
        double vy = rot[1];
        double omega = rx * rotateScale;

        driveRobotFrame(vx, vy, omega);
    }

    /* ---------------- drive with target bearing vector (auton) ---------------- */
    public void drive(Vec2 fieldVelocity, double headingTargetDeg,
                      double rotKP) {

        // translation: field → robot frame
        double[] rot = rotateFieldToRobot(fieldVelocity.x,
                fieldVelocity.y,
                Math.toDegrees(chassisHeadingRad));
        double vx = rot[0];
        double vy = rot[1];

        // rotation: heading error → ω   (quick P controller for now)
        double errDeg = Utils.wrapAngle(headingTargetDeg
                - Math.toDegrees(chassisHeadingRad));
        double omega = (errDeg / 180.0) * rotKP;   // normalised output

        driveRobotFrame(vx, vy, omega);
    }

    /* ---------------- periodic update ---------------- */

    @Override
    public void update(Canvas fieldOverlay) {
        imuAngles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        chassisHeadingRad = imuAngles.firstAngle + Math.toRadians(gyroOffset);

        // keep PID steering loops running every cycle
        back.update();
        left.update();
        right.update();

        drawRobot(fieldOverlay);
    }


    /* ---------------- helper: draw on dashboard (optional) ---------------- */

    private void drawRobot(Canvas c) {
        // TODO: use center + pos[i] to plot wheel vectors if desired
    }

    /* ---------------- circum-centre closed-form (three points) ---------------- */

    private static Vec2 circumCenter(Vec2 p1, Vec2 p2, Vec2 p3) {
        double d = 2 * (p1.x * (p2.y - p3.y) +
                p2.x * (p3.y - p1.y) +
                p3.x * (p1.y - p2.y));
        if (Math.abs(d) < 1e-6) return new Vec2(0,0); // nearly colinear – fall back to origin

        double x = ((sq(p1)* (p2.y - p3.y)) +
                (sq(p2)* (p3.y - p1.y)) +
                (sq(p3)* (p1.y - p2.y))) / d;
        double y = ((sq(p1)* (p3.x - p2.x)) +
                (sq(p2)* (p1.x - p3.x)) +
                (sq(p3)* (p2.x - p1.x))) / d;
        return new Vec2(x, y);
    }
    private static double sq(Vec2 v) { return v.x*v.x + v.y*v.y; }

    /* ---------------- tiny 2-D vector helper ---------------- */

    private static class Vec2 {
        final double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
        Vec2 minus(Vec2 o)       { return new Vec2(x - o.x, y - o.y); }
        double dist(Vec2 o)      { return Math.hypot(x - o.x, y - o.y); }
    }

    /** Field-to-robot frame rotation (degrees in, components out). */
    private static double[] rotateFieldToRobot(double vxField,
                                               double vyField,
                                               double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double vxRobot =  vxField * cos + vyField * sin;
        double vyRobot = -vxField * sin + vyField * cos;
        return new double[]{vxRobot, vyRobot};
    }

    /* ---------------- unused Subsystem boilerplate ---------------- */

    @Override public void stop()          { /* stop motors if desired */ }
    @Override public void resetStates()   { }
    @Override public Map<String, Object> getTelemetry(boolean debug) {
        Map<String,Object> t = new HashMap<>();
        t.put("headingDeg", chassisHeadingRad * 180.0/Math.PI);
        t.put("centreXmm",  center.x);
        t.put("centreYmm",  center.y);
        t.put("backAngle",  back.getTargetAngle());
        t.put("leftAngle",  left.getTargetAngle());
        t.put("rightAngle", right.getTargetAngle());
        return t;
    }
    @Override public String getTelemetryName() { return "IsoSwerve"; }
}
