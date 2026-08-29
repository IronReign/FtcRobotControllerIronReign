package org.firstinspires.ftc.teamcode.robots.newBot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;

public interface Subsystem {

    void readSensors();

    void calc(Canvas fieldOverlay);

    void act();

    void stop();

    void resetStates();
}