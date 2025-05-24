//package org.firstinspires.ftc.teamcode.robots.swervolicious;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//@Autonomous(name = "ROBORAMAAUTO")
//public class RoboRamaAuto extends OpMode {
//    @Override
//    public void init() {
//
//    }
//
//    @Override
//    public void loop() {
//
//    }
//
//    public enum GameState {
//        //MAIN OPMODES
//        AUTONOMOUS("Autonomous", true), TELE_OP("Tele-Op"),
//
//        //TEST & TEMP MODES
//        TEST("Test"),
//        AUTO_SPECIMEN("Auto-Specimens", true),
//        AUTO_SPECIMEN_SWEEP("Auto-Spec-Sweep", true),
//        DEMO("Demo"),
//        MANUAL_DIAGNOSTIC("Manual Diagnostic"), SQUARE("Square"), TURN("Turn"),
//        RELOCALIZATION_TEST("Relocalize");
//        private final String name;
//        private final boolean autonomous;
//
//        GameState(String name, boolean autonomous) {
//            this.name = name;
//            this.autonomous = autonomous;
//        }
//
//        GameState(String name) {
//            this(name, false);
//        }
//
//        public static GameState getGameState(int gameStateIndex) {
//            switch (gameStateIndex) {
//                case 0:
//                    return IntoTheSwerve_6832.GameState.AUTONOMOUS;
//                case 1:
//                    return IntoTheSwerve_6832.GameState.TELE_OP;
//                case 2:
//                    return IntoTheSwerve_6832.GameState.TEST;
//                case 3:
//                    return IntoTheSwerve_6832.GameState.AUTO_SPECIMEN;
//                case 4:
//                    return IntoTheSwerve_6832.GameState.AUTO_SPECIMEN_SWEEP;
//                case 5:
//                    return IntoTheSwerve_6832.GameState.RELOCALIZATION_TEST;
//                case 6:
//                    return IntoTheSwerve_6832.GameState.DEMO;
//                default:
//                    return IntoTheSwerve_6832.GameState.TEST;
//            }
//        }
//
//    }
//}
//
