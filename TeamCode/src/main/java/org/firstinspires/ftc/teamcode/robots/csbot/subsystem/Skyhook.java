package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.csbot.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;


@Config(value = "AA_CS_SKYHOOK")
public class Skyhook implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;
    public static int skyhookRightTicks = 0;
    public static int skyhookLeftTicks = 0;
    public static int SKYHOOK_HANG_TICKS = 300;
    public static int SKYHOOK_LAUNCH_TICKS = 340;
    public static int PREP_FOR_HANG_TICKS = 0;
    public int droneServoTicks = 1500;
    public static int DRONE_TENSION_TICKS = 1450;
    public static int DRONE_RELEASE_TICKS = 750;
    public static int SKYHOOK_SAFE_TICKS = 900;
    public static int SKYHOOK_UP_TICKS = 0;
    DcMotorEx kareem, jabbar;
    IMU skyhookIMU = null;
    public DcMotorExResetable skyhookLeft, skyhookRight;
    public Servo droneLauncher;
    public static int SKYHOOK_INIT_TICKS = 500;

    public static double SKYHOOK_POWER = 1;

    public enum Articulation {
        HANG,
        LAUNCH,
        INIT,
        GAME,
        MANUAL,
        PREP_FOR_HANG,
        UP
    }

    public Articulation articulation = Articulation.MANUAL;

    public Skyhook(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        initMotors();
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        droneServoTicks = DRONE_TENSION_TICKS;
        skyhookIMU = hardwareMap.get(IMU.class, "skyhookIMU");
        //skyhookRightTicks = 0;
        //skyhookLeftTicks = 0;
    }

    public Articulation articulate(Articulation target) {
        this.articulation = target;
        return articulation;
    }

    @Override
    public void update(Canvas fieldOverlay) {

        switch (articulation) {
            case PREP_FOR_HANG:
                skyhookLeftTicks = PREP_FOR_HANG_TICKS;
                skyhookRightTicks = PREP_FOR_HANG_TICKS;
                break;
            case GAME:
                skyhookRightTicks = SKYHOOK_SAFE_TICKS;
                skyhookLeftTicks = SKYHOOK_SAFE_TICKS;
                droneServoTicks = DRONE_TENSION_TICKS;
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
            skyhookRight.setTargetPosition(skyhookRightTicks);
            skyhookLeft.setTargetPosition(skyhookLeftTicks);
        }

        skyhookLeft.setPower(SKYHOOK_POWER);
        skyhookRight.setPower(SKYHOOK_POWER);

    }

    public static long launchTimer = 0;
    public static int launchIndex = 0;
    public static double SKYHOOK_LAUNCH_TIMER = 2;
    public boolean launch() {
        droneLauncher.setPosition(Utils.servoNormalize(droneServoTicks));
        skyhookRight.setTargetPosition(skyhookRightTicks);
        skyhookLeft.setTargetPosition(skyhookLeftTicks);
        switch (launchIndex) {
            case 0:
                launchTimer = futureTime(SKYHOOK_LAUNCH_TIMER);
                launchIndex ++;
                break;
            case 1:
                skyhookRightTicks = SKYHOOK_LAUNCH_TICKS;
                skyhookLeftTicks = SKYHOOK_LAUNCH_TICKS;
                if(withinError(skyhookIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES), 132, 3)) {
                    skyhookLeftTicks = getSkyhookLeftTicksCurrent();
                    skyhookRightTicks = getSkyhookRightTicksCurrent();
                    launchTimer = futureTime(.7);
                    launchIndex ++;
                }
                break;
            case 2:
                if(isPast(launchTimer)) {
                    releaseTheDrone();
                    launchTimer = futureTime(SKYHOOK_LAUNCH_TIMER);
                    launchIndex++;
                }
                break;
            case 3:
                if(isPast(launchTimer)){
                    resetDrone();
                    launchIndex++;
                }
                break;
            case 4:
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

        telemetryMap.put("skyhook right amps", skyhookRight.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("skyhook left amps", skyhookLeft.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("articulation", articulation);
        telemetryMap.put("skyhookLeftTicks", skyhookLeftTicks);
        telemetryMap.put("skyhookLeftActual", skyhookLeft.getCurrentPosition());
        telemetryMap.put("kareemActual", kareem.getCurrentPosition());
        telemetryMap.put("skyhookLeftTarget", skyhookLeft.getTargetPosition());
        telemetryMap.put("kareemTarget", kareem.getTargetPosition());
        telemetryMap.put("skyhookRightActual", skyhookRight.getCurrentPosition());
        telemetryMap.put("jabbarActual", jabbar.getCurrentPosition());
        telemetryMap.put("droneTicks", droneServoTicks);
        telemetryMap.put("droneStage", launchIndex);
//        telemetryMap.put("skyhookIMU", skyhookIMU.getRobotYawPitchRollAngles());
        telemetryMap.put("Skyhook Left Memory Position", robot.positionCache.readPose().getSkyhookLeftTicks());
        telemetryMap.put("Skyhook Right Memory Position", robot.positionCache.readPose().getSkyhookRightTicks());

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
