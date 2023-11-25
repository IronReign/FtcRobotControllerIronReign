package org.firstinspires.ftc.teamcode.robots.goldenduck;

//@Config ("GoldenDuckGameVariables")
//@TeleOp(name="Golden Duck OpMode", group="Challenge")
//public class GoldenDuckOpMode extends OpMode {
//    public boolean auton = true;
//    public static boolean testing = false;
//    public static boolean red = true;
//
//    public static boolean calibrateOn = true;
//    private boolean calibrate = false;
//    public static float DEADZONE = .1f;
//    static final double FEET_PER_METER = 3.28084;
//    int tagDetected = 0;
//
//    Robot ggd;
//    @Override
//    public void init() {
//      ggd = new Robot(telemetry, hardwareMap);
//        servoRailgun = hardwareMap.get(Servo.class,"servoRailgun");
//        servoClaw = hardwareMap.get(Servo.class, "claw");
//        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
//    }
//
//    @Override
//    public void init_loop() {
//        arm();
//        claws();
//        clawWrist();
//        setServoRailgun();
//        telemetry.update();
//    }
//
//
