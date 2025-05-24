package org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.TelemetryProvider;

/**
 * @author Mahesh Natamai
 */

public interface Subsystem extends TelemetryProvider {
    void update(Canvas fieldOverlay);
    void stop();
    void resetStates();
    boolean calibrate();

}
