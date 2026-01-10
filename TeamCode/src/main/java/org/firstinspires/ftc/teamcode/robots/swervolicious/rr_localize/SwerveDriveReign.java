package org.firstinspires.ftc.teamcode.robots.swervolicious.rr_localize;

import static org.firstinspires.ftc.teamcode.robots.swervolicious.rr_localize.SwerveDriveReign.PARAMS;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.*;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.rrQuickStart.Localizer;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.SwerveModule;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.lang.Math;
import java.util.Arrays;

/**
 * SwerveDriveReign — Road Runner drive class for the three‑module “isosceles‑triangle” swerve.
 */
//@Config
public class SwerveDriveReign {

    /* ============================ PARAMETERS ============================ */
    public static class Params {
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public RevHubOrientationOnRobot.UsbFacingDirection  usbFacingDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        public Vector2d backPosMM  = new Vector2d(-191.029,   0.000);
        public Vector2d leftPosMM  = new Vector2d( 106.754,  -158.500);
        public Vector2d rightPosMM = new Vector2d( 106.754, 158.500);
        public double   inPerTick  = 0.00293063133; // convert SwerveModule ticks of drive motor into travel
        public double   degPerTick = 360.0 / 4920.0; // convert SwerveModule through bore encoder ticks to degrees of steering
        public double   maxWheelVel = 50;  // in/s
        public double   minProfileAccel = -30;
        public double   maxProfileAccel = 50;
        public double   maxAngVel   = Math.PI;
        public double maxAngAccel = Math.PI;
        // PID parameters (adjust as needed)
        public PIDCoefficients pidCoefficients = new PIDCoefficients(.045, 0, 0.5);
        // Set a threshold (in degrees) below which the drive motor is allowed to run.
        public double yawThreshold = 10; // suppress drive until steering is within this threshold


    }
    public static Params PARAMS = new Params();

    /* ============================ KINEMATICS ============================ */
    public final IsoSwerveKinematics kinematics = new IsoSwerveKinematics(PARAMS);

    /* ============================ CONSTRAINTS =========================== */
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
           new AngularVelConstraint(PARAMS.maxAngVel);
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);



    /* ============================ HARDWARE ============================== */
    private final SwerveModule backModule, leftModule, rightModule;
    private final VoltageSensor voltageSensor;
    private final LazyImu      lazyImu;
    public final Localizer    localizer;
    // --- pose history for dashboard drawing
    private final java.util.LinkedList<Pose2d> poseHistory = new java.util.LinkedList<>();


    public SwerveDriveReign(HardwareMap hw, Pose2d startPose){
        for(LynxModule m: hw.getAll(LynxModule.class)) m.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        backModule  = SwerveModuleFactory.make("back","go0","yaw0","encoder0","a0",hw);
        leftModule  = SwerveModuleFactory.make("left","go1","yaw1","encoder1","a1",hw);
        rightModule = SwerveModuleFactory.make("right","go2","yaw2","encoder2","a2",hw);
        voltageSensor = hw.voltageSensor.iterator().next();
        lazyImu = new LazyHardwareMapImu(hw,"imu",new RevHubOrientationOnRobot(PARAMS.logoFacingDirection,PARAMS.usbFacingDirection));
        //localizer = new SwerveDriveReignLocalizer(startPose);
        localizer = new PinpointLocalizer(hw,PARAMS.inPerTick, startPose);
    }

    /* ======================== TELE‑OP POWER SETTER ===================== */
    public void setDrivePowers(PoseVelocity2d cmd){
        IsoSwerveKinematics.WheelVelocities wheel = kinematics.inverse(cmd);
        double max = wheel.maxMag;
        if(max<1e-6) max=1; // avoid /0
        backModule .setDesiredState(wheel.bAngDeg, wheel.bVel/max);
        leftModule .setDesiredState(wheel.lAngDeg, wheel.lVel/max);
        rightModule.setDesiredState(wheel.rAngDeg, wheel.rVel/max);

        boolean allAligned = backModule.swerveAligned &&
                leftModule.swerveAligned &&
                rightModule.swerveAligned;

        if (!allAligned) {
            backModule.driveMotor.setPower(0);
            leftModule.driveMotor.setPower(0);
            rightModule.driveMotor.setPower(0);
        }
    }

    /* ============================= LOCALIZER (based on swerve steering and drive for this specific chassis =========================== */
    private class SwerveDriveReignLocalizer implements Localizer{
        private Pose2d pose;
        private final IMU imu = lazyImu.get();
        private final Encoder bEnc,lEnc,rEnc,bSteer,lSteer,rSteer;
        private int lastB,lastL,lastR;
        SwerveDriveReignLocalizer(Pose2d p){ pose=p;
            bEnc=new OverflowEncoder(new RawEncoder((DcMotorEx)backModule.getDriveMotor()));
            lEnc=new OverflowEncoder(new RawEncoder((DcMotorEx)leftModule.getDriveMotor()));
            rEnc=new OverflowEncoder(new RawEncoder((DcMotorEx)rightModule.getDriveMotor()));
            bSteer=new OverflowEncoder(new RawEncoder(backModule.getYawEncoder()));
            lSteer=new OverflowEncoder(new RawEncoder(leftModule.getYawEncoder()));
            rSteer=new OverflowEncoder(new RawEncoder(rightModule.getYawEncoder()));}
        @Override public void setPose(Pose2d p){pose=p;} @Override public Pose2d getPose(){return pose;}
        @Override public PoseVelocity2d update(){
            PositionVelocityPair bp=bEnc.getPositionAndVelocity();
            PositionVelocityPair lp=lEnc.getPositionAndVelocity();
            PositionVelocityPair rp=rEnc.getPositionAndVelocity();
            double bA=Math.toRadians(bSteer.getPositionAndVelocity().position*PARAMS.degPerTick);
            double lA=Math.toRadians(lSteer.getPositionAndVelocity().position*PARAMS.degPerTick);
            double rA=Math.toRadians(rSteer.getPositionAndVelocity().position*PARAMS.degPerTick);
            Vector2d db=new Vector2d(Math.cos(bA),Math.sin(bA)).times(ticksToInches(bp.position-lastB));
            Vector2d dl=new Vector2d(Math.cos(lA),Math.sin(lA)).times(ticksToInches(lp.position-lastL));
            Vector2d dr=new Vector2d(Math.cos(rA),Math.sin(rA)).times(ticksToInches(rp.position-lastR));
            lastB=bp.position; lastL=lp.position; lastR=rp.position;
            IsoSwerveKinematics.Twist twist=kinematics.forward(db,dl,dr);
            pose = pose.plus(new Twist2d(new Vector2d(twist.vx, twist.vy), twist.w));
            return new PoseVelocity2d(new Vector2d(twist.vx,twist.vy),twist.w);
        }
    }

    /* ========================== KINEMATICS CLASS ======================= */
    public static class IsoSwerveKinematics{
        private final Vector2d rB,rL,rR; private final double maxR;
        IsoSwerveKinematics(Params p){double f=1/25.4; rB=p.backPosMM.times(f); rL=p.leftPosMM.times(f); rR=p.rightPosMM.times(f); maxR=Math.max(Math.max(rB.norm(),rL.norm()),rR.norm());}
        /* result holder */
        public static class WheelVelocities{ public final double bVel,lVel,rVel,bAngDeg,lAngDeg,rAngDeg,maxMag; WheelVelocities(double bv,double lv,double rv,double ba,double la,double ra){bVel=bv;lVel=lv;rVel=rv;bAngDeg=ba;lAngDeg=la;rAngDeg=ra;maxMag=Math.max(Math.max(Math.abs(bv),Math.abs(lv)),Math.abs(rv));}}
        public static class Twist{ public final double vx,vy,w; Twist(double vx,double vy,double w){this.vx=vx;this.vy=vy;this.w=w;}}
        /* inverse: chassis -> wheels */
        public WheelVelocities inverse(PoseVelocity2d v){ double vx=v.linearVel.x, vy=v.linearVel.y, w=v.angVel;
            double bX=vx-w*rB.y/maxR, bY=vy+w*rB.x/maxR; double lX=vx-w*rL.y/maxR, lY=vy+w*rL.x/maxR; double rX=vx-w*rR.y/maxR, rY=vy+w*rR.x/maxR;
            double bS=Math.hypot(bX,bY), lS=Math.hypot(lX,lY), rS=Math.hypot(rX,rY);
            double bA=Math.toDegrees(Math.atan2(bY,bX)), lA=Math.toDegrees(Math.atan2(lY,lX)), rA=Math.toDegrees(Math.atan2(rY,rX));
            return new WheelVelocities(bS,lS,rS,bA,lA,rA);
        }
        /* forward: wheels -> chassis (least‑squares) */
        public Twist forward(Vector2d db,Vector2d dl,Vector2d dr){
            // 6x3 normal eq solve closed‑form
            double[][] A={{1,0,-rB.y/maxR},{0,1,rB.x/maxR},{1,0,-rL.y/maxR},{0,1,rL.x/maxR},{1,0,-rR.y/maxR},{0,1,rR.x/maxR}};
            double[] b={db.x,db.y,dl.x,dl.y,dr.x,dr.y};
            double[][] AtA=new double[3][3]; double[] Atb=new double[3];
            for(int i=0;i<6;i++){AtA[0][0]+=A[i][0]*A[i][0];AtA[0][1]+=A[i][0]*A[i][1];AtA[0][2]+=A[i][0]*A[i][2];AtA[1][1]+=A[i][1]*A[i][1];AtA[1][2]+=A[i][1]*A[i][2];AtA[2][2]+=A[i][2]*A[i][2];Atb[0]+=A[i][0]*b[i];Atb[1]+=A[i][1]*b[i];Atb[2]+=A[i][2]*b[i];}
            AtA[1][0]=AtA[0][1];AtA[2][0]=AtA[0][2];AtA[2][1]=AtA[1][2];
            double det=AtA[0][0]*(AtA[1][1]*AtA[2][2]-AtA[1][2]*AtA[1][2])-AtA[0][1]*(AtA[0][1]*AtA[2][2]-AtA[0][2]*AtA[1][2])+AtA[0][2]*(AtA[0][1]*AtA[1][2]-AtA[0][2]*AtA[1][1]);
            if(Math.abs(det)<1e-6) return new Twist(0,0,0);
            double inv00=(AtA[1][1]*AtA[2][2]-AtA[1][2]*AtA[1][2])/det; double inv01=(AtA[0][2]*AtA[1][2]-AtA[0][1]*AtA[2][2])/det; double inv02=(AtA[0][1]*AtA[1][2]-AtA[0][2]*AtA[1][1])/det;
            double inv11=(AtA[0][0]*AtA[2][2]-AtA[0][2]*AtA[0][2])/det; double inv12=(AtA[0][2]*AtA[0][1]-AtA[0][0]*AtA[1][2])/det; double inv22=(AtA[0][0]*AtA[1][1]-AtA[0][1]*AtA[0][0])/det;
            double vx=inv00*Atb[0]+inv01*Atb[1]+inv02*Atb[2]; double vy=inv01*Atb[0]+inv11*Atb[1]+inv12*Atb[2]; double w=inv02*Atb[0]+inv12*Atb[1]+inv22*Atb[2];
            return new Twist(vx,vy,w);
        }

    }

    /** Lightweight holder for steering telemetry (radians). */
    public static class ModuleState {
        public final double currentAngle;
        public final double targetAngle;
        public ModuleState(double currentRad, double targetRad) {
            this.currentAngle = currentRad;
            this.targetAngle = targetRad;
        }
    }

    /**
     * Returns the current vs. target angle for each swerve module
     * in the order: back, left, right (radians).
     * <p>
     * <b>Implementation notes:</b> This assumes each {@code SwerveModule}
     * exposes {@code getCurrentAngleDeg()} and {@code getTargetAngleDeg()}.
     * Rename these calls if your API differs.
     */
    public ModuleState[] getModuleStates() {
        return new ModuleState[] {
                new ModuleState(Math.toRadians(backModule.getCurrentAngle()),  Math.toRadians(backModule.getTargetAngle())),
                new ModuleState(Math.toRadians(leftModule.getCurrentAngle()),  Math.toRadians(leftModule.getTargetAngle())),
                new ModuleState(Math.toRadians(rightModule.getCurrentAngle()), Math.toRadians(rightModule.getTargetAngle()))
        };
    }


    /* ============================ HELPERS ============================== */
    private static double ticksToInches(int t){return t*PARAMS.inPerTick;}

    /**
     * Wrapper that many of our op modes expect. Calls the localizer, stores a short
     * pose history for drawing, and returns the velocity for logging.
     */
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        while (poseHistory.size() > 100) poseHistory.removeFirst();
        return vel;
    }

    /** Call every robot loop to keep each module’s yaw PID active. */
    public void updateModules(Canvas c) {
        backModule.update(c);
        leftModule.update(c);
        rightModule.update(c);
    }

    /** Draws a breadcrumb trail of the recent robot poses on the FTC Dashboard. */
    private void drawPoseHistory(Canvas c) {
        double[] xs = new double[poseHistory.size()];
        double[] ys = new double[poseHistory.size()];
        int i = 0;
        for (Pose2d p : poseHistory) { xs[i] = p.position.x; ys[i] = p.position.y; i++; }
        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xs, ys);
    }

    /** Convenience builder so autonomous code can keep using the same call site. */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,        // we reuse Mecanum‑style inner actions, adapted below
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint);
    }
    /* --- Minimal drivetrain‑specific Action replacements -------------------- */

    public final class FollowTrajectoryAction implements Action {
        private final TimeTrajectory traj; private double start = -1;
        public FollowTrajectoryAction(TimeTrajectory t){ traj=t; }
        @Override public boolean run(@NonNull TelemetryPacket pkt){
            double t = (start < 0) ? 0 : Actions.now() - start; if(start<0){start=Actions.now();}
            if(t >= traj.duration){ backModule.aligned(0); leftModule.aligned(0); rightModule.aligned(0); return false; }
            Pose2dDual<Time> target = traj.get(t);
            PoseVelocity2d actualVel = updatePoseEstimate();
            PoseVelocity2dDual<Time> cmd = new HolonomicController(5,10,10,0,0,0.005)
                    .compute(target, localizer.getPose(), actualVel);
            IsoSwerveKinematics.WheelVelocities wheel = kinematics.inverse(cmd.value());
            setDrivePowers(cmd.value()); // uses existing normalisation helper
            Canvas c = pkt.fieldOverlay(); drawPoseHistory(c); return true; }
        @Override public void preview(Canvas c) { /* no-op */ }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn; private double start=-1;
        public TurnAction(TimeTurn t){turn=t;}
        @Override public boolean run(@NonNull TelemetryPacket pkt){
            double t=(start<0)?0:Actions.now()-start; if(start<0){start=Actions.now();}
            if(t>=turn.duration){ setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0)); return false; }
            Pose2dDual<Time> target=turn.get(t);
            PoseVelocity2dDual<Time> cmd=new HolonomicController(5,10,10,0,0,0.005)
                    .compute(target, localizer.getPose(), updatePoseEstimate());
            setDrivePowers(cmd.value()); Canvas c=pkt.fieldOverlay(); drawPoseHistory(c); return true; }
        @Override public void preview(Canvas c) { /* no-op */ }
    }
}

/* ---------------- SwerveModule factory -------------------------------- */
class SwerveModuleFactory{
    static SwerveModule make(String name, String drive,String yawMotor,String yawEnc,String yawIndex, HardwareMap hw){
        DcMotorEx d=hw.get(DcMotorEx.class,drive);
        CRServo y=hw.get(CRServo.class,yawMotor);
        DcMotorEx e=hw.get(DcMotorEx.class,yawEnc);
        AnalogInput a=hw.get(AnalogInput.class, yawIndex);
        PIDController pid=new PIDController(PARAMS.pidCoefficients);
        pid.setOutputRange(-0.5, 0.5);
        pid.setContinuous();
        pid.setTolerance(50);
        pid.setInputRange(0, 360);
        pid.enable();
        return new SwerveModule(name,d,y,e,a,pid,1/ PARAMS.degPerTick,PARAMS.yawThreshold);
    }
}
