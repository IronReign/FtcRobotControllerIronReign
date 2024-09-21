package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.csbot.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;


@Config(value = "AA_CS_SKYHOOK")
public class Skyhook implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;
    public static int skyhookRightTicks = 0;
    public static int skyhookLeftTicks = 0;
    public static int SKYHOOK_HANG_TICKS = 450;
    public static int SKYHOOK_LAUNCH_TICKS = 560;
    public static boolean droneLoaded = true;
    public static int PREP_FOR_HANG_TICKS = 0;
    public int droneServoTicks = 1500;
    public static int DRONE_TENSION_TICKS = 1200;
    public static int DRONE_RELEASE_TICKS = 2000;
    public static int SKYHOOK_SAFE_TICKS = 1200;
    public static int SKYHOOK_UP_TICKS = 0;
    public PIDController dronelaunchPID;
    public static PIDCoefficients DRONELAUNCH_PID_PWR = new PIDCoefficients(0.03, 0.04, 0);
    public static double DRONELAUNCH_PID_TOLERANCE = .08;
    private double PIDCorrection, PIDError;
    public static double DRONE_LAUNCH_ANGLE = 57;
    DcMotorEx kareem, jabbar;
    IMU skyhookIMU;
    RevTouchSensor rightTouchSensor, leftTouchSensor;
    public DcMotorExResetable skyhookLeft, skyhookRight;
    public Servo droneLauncher;
    public static int SKYHOOK_INIT_TICKS = 800;

    public static double SKYHOOK_POWER = 1;
    private boolean manualDrone = true;

    public enum Articulation {
        HANG,
        LAUNCH,
        INIT,
        TRAVEL,
        MANUAL,
        PREP_FOR_HANG,
        UP
    }

    public Articulation articulation = Articulation.MANUAL;

    public Skyhook(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        initMotors();
        leftTouchSensor = hardwareMap.get(RevTouchSensor.class, "leftTouchSensor");
        rightTouchSensor = hardwareMap.get(RevTouchSensor.class, "rightTouchSensor");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        droneServoTicks = DRONE_TENSION_TICKS;
        skyhookIMU = hardwareMap.get(IMU.class, "skyhookIMU");
        dronelaunchPID = new PIDController(DRONELAUNCH_PID_PWR);
        dronelaunchPID.setInputRange(0, 360);
        dronelaunchPID.setOutputRange(-1, 1);
        dronelaunchPID.setIntegralCutIn(4);
        dronelaunchPID.setContinuous(true);
        dronelaunchPID.setTolerance(DRONELAUNCH_PID_TOLERANCE);
        dronelaunchPID.enable();
        //skyhookRightTicks = 0;
        //skyhookLeftTicks = 0;
    }

    public Articulation articulate(Articulation target) {
        this.articulation = target;
        return articulation;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        if(manualDrone) {
            if (droneLoaded)
                droneServoTicks = DRONE_TENSION_TICKS;
            else
                droneServoTicks = DRONE_RELEASE_TICKS;
        }
        switch (articulation) {
            case PREP_FOR_HANG:
                skyhookLeftTicks = PREP_FOR_HANG_TICKS;
                skyhookRightTicks = PREP_FOR_HANG_TICKS;
                break;
            case TRAVEL:
                skyhookRightTicks = SKYHOOK_SAFE_TICKS;
                skyhookLeftTicks = SKYHOOK_SAFE_TICKS;
                break;
            case LAUNCH:
                if(launch()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            case MANUAL:
                break;
            case HANG:
                if(hang()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            case INIT:
                skyhookRightTicks = SKYHOOK_INIT_TICKS;
                skyhookLeftTicks = SKYHOOK_INIT_TICKS;
                droneServoTicks = DRONE_TENSION_TICKS;
                articulation = articulation.MANUAL;
                break;
        }
        droneLauncher.setPosition(Utils.servoNormalize(droneServoTicks));
        if(skyhookRight.getCurrent(CurrentUnit.AMPS) > 3.5 || skyhookLeft.getCurrent(CurrentUnit.AMPS) > 3.5){
            SKYHOOK_POWER = 0;
            articulation = Articulation.MANUAL;
        }else {
            if(!robot.initing) {
                skyhookRight.setTargetPosition(skyhookRightTicks);
                skyhookLeft.setTargetPosition(skyhookLeftTicks);
            }
        }
        if(!robot.initing) {
            skyhookLeft.setPower(SKYHOOK_POWER);
            skyhookRight.setPower(SKYHOOK_POWER);
        }
    }

    public static long launchTimer = 0;
    public static int launchIndex = 0;
    public static double SKYHOOK_LAUNCH_TIMER = 3;
    public boolean launch() {
        droneLauncher.setPosition(Utils.servoNormalize(droneServoTicks));
        skyhookRight.setTargetPosition(skyhookRightTicks);
        skyhookLeft.setTargetPosition(skyhookLeftTicks);
        switch (launchIndex) {
            case 0:
                manualDrone = false;
                launchTimer = futureTime(SKYHOOK_LAUNCH_TIMER);
                Sensors.skyhookIMUEnabled = true;
                launchIndex ++;
                break;
            case 1:
                skyhookRightTicks = SKYHOOK_LAUNCH_TICKS;
                skyhookLeftTicks = SKYHOOK_LAUNCH_TICKS;
                launchIndex ++;
//                launchIndex ++;
//                dronelaunchPID.setPID(DRONELAUNCH_PID_PWR);
//                dronelaunchPID.setInput(wrapAngle(Robot.sensors.skyhookIMUPitch));
//                dronelaunchPID.setSetpoint(DRONE_LAUNCH_ANGLE);
//                dronelaunchPID.setOutputRange(-.8, .8);
//                dronelaunchPID.setTolerance(DRONELAUNCH_PID_TOLERANCE);
//                double correction = dronelaunchPID.performPID();
//                PIDCorrection = correction;
//                PIDError = dronelaunchPID.getError();
//                if (dronelaunchPID.onTarget()) {
////                    skyhookRight.setVelocity();
////                    skyhookLeft.setVelocity();
//                    Sensors.skyhookIMUEnabled = false;
//                } else {
//                    dronelaunchPID.enable();
////                    skyhookRight.setVelocity();
////                    skyhookLeft.setVelocity();
//                }
                break;
            case 2:
                if(isPast(launchTimer)) {
                    releaseTheDrone();
                    droneLoaded = false;
//                    Sensors.skyhookIMUEnabled = false;
                    launchTimer = futureTime(SKYHOOK_LAUNCH_TIMER);
                    launchIndex++;
                }
                break;
            case 3:
//                if(isPast(launchTimer)){
//                    resetDrone();
                    launchIndex++;
//                }
                break;
            case 4:
                manualDrone = true;
                launchIndex = 0;
                return true;

        }
        return false;
    }

    public boolean hang() {
        skyhookLeftTicks = SKYHOOK_HANG_TICKS;
        skyhookRightTicks = SKYHOOK_HANG_TICKS;
        return true;
    }

    public void releaseTheDrone() {
        droneServoTicks = DRONE_RELEASE_TICKS;
    }

    public void resetDrone(){
        droneServoTicks = DRONE_TENSION_TICKS;
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("leftTouch", robot.sensors.leftTouchSensor);
        telemetryMap.put("rightTouch", robot.sensors.rightTouchSensor);

        telemetryMap.put("skyhook right amps", skyhookRight.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("skyhook left amps", skyhookLeft.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("articulation", articulation);
        telemetryMap.put("skyhookLeftTicks", skyhookLeftTicks);
        telemetryMap.put("skyhookLeftActual", skyhookLeft.getCurrentPosition());
        telemetryMap.put("skyhookLeftTarget", skyhookLeft.getTargetPosition());
        telemetryMap.put("skyhookRightTicks", skyhookRightTicks);
        telemetryMap.put("skyhookRightActual", skyhookRight.getCurrentPosition());
        telemetryMap.put("skyhookRightTarget", skyhookRight.getTargetPosition());
        telemetryMap.put("droneTicks", droneServoTicks);
        telemetryMap.put("droneStage", launchIndex);
        telemetryMap.put("skyhookIMU", robot.sensors.skyhookIMUPitch);
//        telemetryMap.put("Skyhook Left Memory Position", robot.positionCache.readPose().getSkyhookLeftTicks());
//        telemetryMap.put("Skyhook Right Memory Position", robot.positionCache.readPose().getSkyhookRightTicks());

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "SKYHOOK";
    }

    public void initMotors() {
        kareem = this.hardwareMap.get(DcMotorEx.class,"kareem");
        kareem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skyhookLeft = new DcMotorExResetable(kareem);
        jabbar = this.hardwareMap.get(DcMotorEx.class, "jabbar");
        jabbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skyhookRight = new DcMotorExResetable(jabbar);
        skyhookLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        skyhookLeftTicks = skyhookLeft.getCurrentPosition();
        skyhookLeft.setTargetPosition(skyhookLeftTicks);
        skyhookLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skyhookLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        skyhookRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skyhookRightTicks = skyhookRight.getCurrentPosition();
        skyhookRight.setTargetPosition(skyhookRightTicks);
        skyhookRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        droneServoTicks = DRONE_TENSION_TICKS;
        SKYHOOK_POWER = 1;
        skyhookLeft.setPower(1);
        skyhookRight.setPower(1);
    }

    public int getSkyhookLeftTicksCurrent(){return skyhookLeft.getCurrentPosition();}
    public int getSkyhookRightTicksCurrent(){return skyhookRight.getCurrentPosition();}
}
