package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.config.Config;

@Config("BoboAutonVariables")
abstract class Assign {
    public static int TICKSPERTILE = 3055;
    public static float MAXMOTORSPEED = 1.0f;
    public static double TIMEBETWEENTASKS = 1;
    public static final int NANOTOSECOND = 1000000000;
    abstract boolean run();
}
