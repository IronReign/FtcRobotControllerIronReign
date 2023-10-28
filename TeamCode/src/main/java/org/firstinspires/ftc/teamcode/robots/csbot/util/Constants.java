package org.firstinspires.ftc.teamcode.robots.csbot.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config(value = "PPConstants")
public class Constants {


    //Subsystems
    //----------------------------------------------------------------------------------------------


    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    // driveTrain
    public static double FIELD_INCHES_PER_GRID = 23.5;




    //----------------------------------------------------------------------------------------------
    // Control Constants
    //----------------------------------------------------------------------------------------------

    public static double EPSILON = 1e-6; // small value used for the approximately equal calculation in MathUtils
    public static double TRIGGER_DEADZONE = 0.1; // gamepad trigger values below this threshold will be ignored
    public static double JOYSTICK_DEADZONE = 0.05;

    //----------------------------------------------------------------------------------------------
    // Simulation
    //----------------------------------------------------------------------------------------------
    public static boolean USE_MOTOR_SMOOTHING = false;
    public static double INCHES_PER_METER = 39.3701;

    //----------------------------------------------------------------------------------------------
    // Enums
    //----------------------------------------------------------------------------------------------
    public enum Alliance {
        RED(true), BLUE(false);

        private boolean mod;

        Alliance(boolean mod) {
            this.mod = mod;
        }
        public boolean getMod() {
            return mod;
        }
        public void Toggle(){this.mod = !this.mod;}
    }

    public enum Position {
        ORIGIN_DEFAULT (new Pose2d(0, 0, 0)), //this if used will reset the origin to FTC Dashboard's default
        ORIGIN_ALLIANCE_RED (new Pose2d(0, 12*6, 0)), //these origin redefinitions are relative to the FTC Dashboard default which is different from a Canvas default
        ORIGIN_ALLIANCE_BLUE (new Pose2d(0, -12*6, Math.PI)),
        ORIGIN_6CAN (new Pose2d(-5*12, 0, Math.toRadians(0))),
        START_LEFT(new Pose2d(9, 1.5 * FIELD_INCHES_PER_GRID, Math.toRadians(0))),
        START_RIGHT(new Pose2d(9, -1.5 * FIELD_INCHES_PER_GRID, Math.toRadians(0)));
        private final Pose2d pose;

        Position(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

}
