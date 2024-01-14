package org.firstinspires.ftc.teamcode.robots.csbot.util;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.P2D;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config(value = "CSConstants")
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
        START_LEFT_RED(P2D(-1.5, -2.6, -90)),
        START_RIGHT_RED(P2D(.5, -2.6, -90)),
        START_RIGHT_BLUE(P2D(-1.5, 2.6, 90)),
        START_LEFT_BLUE(P2D(.5, 2.6, 90));
        private final Pose2d pose;

        public boolean getMod() {
            return this.name() == START_LEFT_RED.name() || this.name() == START_RIGHT_RED.name() ?  true : false;
        }

        Position(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

}
