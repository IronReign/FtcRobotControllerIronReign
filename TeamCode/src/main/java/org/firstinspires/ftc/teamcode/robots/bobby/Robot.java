package org.firstinspires.ftc.teamcode.robots.bobby;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.bobby.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.bobby.Intake;
import org.firstinspires.ftc.teamcode.robots.bobby.Localizer;
import org.firstinspires.ftc.teamcode.robots.bobby.Launcher;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class Robot implements Subsystem {
    public final DriveTrain drive;
    public final Localizer localization;
    public final Launcher shooter;
    public final Intake intake;

    public final Subsystem[] subsystems;
    public long[] subsystemUpdateTimes;

    public Robot(HardwareMap hardwareMap, double startHeadingRadians) {
        drive = new DriveTrain(hardwareMap);
        localization = new Localizer(hardwareMap, drive, startHeadingRadians);
        shooter = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        subsystems = new Subsystem[]{drive, localization, shooter, intake};
        subsystemUpdateTimes = new long[subsystems.length];
    }

    @Override
    public void update(Canvas fieldOverlay) {
        for (int i = 0; i < subsystems.length; i++) {
            long updateStartTime = System.nanoTime();
            subsystems[i].update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }
    }

    public void handleDrive(Gamepad gamepad) {
        if (gamepad == null) {
            return;
        }
        mecanumDrive(gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x);
        drive.slowTurn(gamepad.dpad_right, gamepad.dpad_left);
    }

    public void mecanumDrive(double forward, double strafe, double turn) {
        drive.mecanumDrive(forward, strafe, turn);
    }

    public void setTurnPower(double turn) {
        drive.mecanumDrive(0, 0, turn);
    }

    public void setFlywheelVelocity(double rpm) {
        shooter.setFlywheelVelocity(rpm);
    }

    public void setIntakePower(double power) {
        intake.setIntakePower(power);
    }

    public void setPusherUp() {
        shooter.setPusherUp();
    }

    public void setPusherDown() {
        shooter.setPusherDown();
    }

    public double getHeading() {
        return localization.getHeading();
    }

    public double getX() {
        return localization.getX();
    }

    public double getY() {
        return localization.getY();
    }

    public void setPose(double x, double y) {
        localization.setPose(x, y);
    }

    public double getRightBackPower() {
        return drive.getRightBackPower();
    }

    @Override
    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }

    @Override
    public void resetStates() {
        for (Subsystem subsystem : subsystems) {
            subsystem.resetStates();
        }
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("x (inches)", localization.getX());
        telemetry.put("y (inches)", localization.getY());
        telemetry.put("heading (rad)", localization.getHeading());
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "bobby robot";
    }
}