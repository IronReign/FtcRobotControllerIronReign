package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.config.Config;

@Config("BoboAutonVariables")
public abstract class Assign {
    public static int TICKSPERTILE = 3055;
    public static int STRAFETICKSPERTILE = 3220;
    public static int TICKSPERDIAG = 1200;
    public static int TICKSPER90 = 1421;
    public static float MAXMOTORSPEED = 1.0f;
    public static double TIMEBETWEENTASKS = 1;
    public static final int NANOTOSECOND = 1000000000;
    public static double delta = 0;
    public abstract double getDelta();
    public abstract boolean run();
}
