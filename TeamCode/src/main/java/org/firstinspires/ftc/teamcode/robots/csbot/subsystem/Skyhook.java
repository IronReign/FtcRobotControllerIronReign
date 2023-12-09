package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;


@Config(value = "CS_SKYHOOK")
public class Skyhook implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;
    public static int skyhookRightTicks = 0;
    public static int skyhookLeftTicks = 0;
    public static int SKYHOOK_HANG_TICKS = 300;
    public static int SKYHOOK_LAUNCH_TICKS = 250;
    public static int jimmyTicks = 1500;
    public static int JIMMY_TENSION_TICKS = 1450;
    public static int JIMMY_RELEASE_TICKS = 2100;
    public static int SKYHOOK_SAFE_TICKS = 900;
    DcMotor kareem, jabbar;
    Servo jimmy;
    public static int SKYHOOK_INIT_TICKS = 900;

    public enum Articulation {
        HANG,
        LAUNCH,
        INIT,
        GAME,
        MANUAL,
        PREP_FOR_HANG
    }

    public Articulation articulation = Articulation.MANUAL;

    public Skyhook(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        initMotors();
        jimmy = hardwareMap.get(Servo.class, "jimmy");
        jimmyTicks = JIMMY_TENSION_TICKS;
        skyhookRightTicks = 0;
        skyhookLeftTicks = 0;
    }

    public Articulation articulate(Articulation target) {
        this.articulation = target;
        return articulation;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        switch (articulation) {
            case PREP_FOR_HANG:
                skyhookLeftTicks = 0;
                skyhookRightTicks = 0;
                articulation = Articulation.MANUAL;
                break;
            case GAME:
                skyhookRightTicks = SKYHOOK_SAFE_TICKS;
                skyhookLeftTicks = SKYHOOK_SAFE_TICKS;
                jimmyTicks = JIMMY_TENSION_TICKS;
                break;
            case LAUNCH:
                if(launch()) {
                    articulation = Articulation.MANUAL;
                }
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
                jimmyTicks = JIMMY_TENSION_TICKS;
                articulation = articulation.MANUAL;
                break;
        }
        jimmy.setPosition(Utils.servoNormalize(jimmyTicks));
        jabbar.setTargetPosition(skyhookRightTicks);
        kareem.setTargetPosition(skyhookLeftTicks);
    }

    public static long launchTimer = 0;
    public static int launchIndex = 0;
    public static double SKYHOOK_LAUNCH_TIMER = 1;
    public boolean launch() {
        jimmy.setPosition(Utils.servoNormalize(jimmyTicks));
        jabbar.setTargetPosition(skyhookRightTicks);
        kareem.setTargetPosition(skyhookLeftTicks);
        switch (launchIndex) {
            case 0:
                launchTimer = futureTime(SKYHOOK_LAUNCH_TIMER);
                launchIndex ++;
                break;
            case 1:
                skyhookRightTicks = SKYHOOK_LAUNCH_TICKS;
                skyhookLeftTicks = SKYHOOK_LAUNCH_TICKS;
                if(isPast(launchTimer)) {
                    launchIndex ++;
                }
                break;
            case 2:
                releaseTheJimmy();
                break;
            case 3:
                return true;

        }
        return false;
    }

    public boolean hang() {
        skyhookLeftTicks = SKYHOOK_HANG_TICKS;
        skyhookRightTicks = SKYHOOK_HANG_TICKS;
        return true;
    }

    public void releaseTheJimmy() {
        jimmyTicks = JIMMY_RELEASE_TICKS;
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("articulation", articulation);
        telemetryMap.put("skyhookLeftTicks", skyhookLeftTicks);
        telemetryMap.put("skyhookRightTicks", skyhookRightTicks);
        telemetryMap.put("jimmyTicks", jimmyTicks);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "SKYHOOK";
    }

    public void initMotors() {
        kareem = this.hardwareMap.get(DcMotorEx.class, "kareem");
        jabbar = this.hardwareMap.get(DcMotorEx.class, "jabbar");
        kareem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kareem.setDirection(DcMotorSimple.Direction.REVERSE);
        kareem.setTargetPosition(0);
        kareem.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        jabbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jabbar.setTargetPosition(0);
        jabbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        kareem.setPower(1);
        jabbar.setPower(1);
    }
}
