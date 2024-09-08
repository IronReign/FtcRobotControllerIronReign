package org.firstinspires.ftc.teamcode.robots.ri2d;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class ri2dbot implements Subsystem {

    public Subsystem [] subsystems;
    public Swerve swerve;
    public HardwareMap hardwareMap;
    private long[] subsystemUpdateTimes;

    public ri2dbot(HardwareMap hardwareMap){
        swerve = new Swerve(hardwareMap,this);
        subsystems = new Subsystem[]{swerve};
    }

    @Override
    public void update(Canvas fieldOverlay) {
        for (int i = 0; i < subsystems.length; i++) {
            Subsystem subsystem = subsystems[i];
            long updateStartTime = System.nanoTime();
            subsystem.update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
        }
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Robot";
    }
}
