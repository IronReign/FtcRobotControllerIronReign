//package org.firstinspires.ftc.teamcode.robots.goldenduck;
//
//import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class placeholder {
//
//@Config("GoldenDuckGameVariables")
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
//
//    DriveTrain driveTrain;
//
//    @Override
//    public void init() {
//        driveTrain = new DriveTrain(telemetry, hardwareMap);
//        driveTrain.motorInit();
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
//    @Override
//    public void loop() {
//        servoRailgun.telemetryOutput();
//        claws();
//    }
//
//
//
//
//    class DriveTrain {
//        private Telemetry telemetry;
//        private HardwareMap hardwareMap;
//        private DcMotorEx motorFrontRight = null;
//        private DcMotorEx motorBackLeft = null;
//        private DcMotorEx motorFrontLeft = null;
//        private DcMotorEx motorBackRight = null;
//        private double powerLeft = 0;
//        private double powerRight = 0;
//        private double powerFrontLeft = 0;
//        private double powerFrontRight = 0;
//        private double powerBackLeft = 0;
//        private double powerBackRight = 0;
//        private static final float DEADZONE = .1f;
//        double robotSpeed = 1;
//
//        public DriveTrain(Telemetry telemetry, HardwareMap hardwareMap) {
//            this.telemetry = telemetry;
//            this.hardwareMap = hardwareMap;
//        }
//
//        public void motorInit() {
//            motorFrontLeft = this.hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
//            motorBackLeft = this.hardwareMap.get(DcMotorEx.class, "motorBackLeft");
//            motorFrontRight = this.hardwareMap.get(DcMotorEx.class, "motorFrontRight");
//            motorBackRight = this.hardwareMap.get(DcMotorEx.class, "motorBackRight");
//            arm = this.hardwareMap.get(DcMotorEx.class, "arm");
//            arm.setTargetPosition(0);
//            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            arm.setPower(0);
//            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motorBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            motorBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            this.motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
//            this.motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
//
//        }
//
//        public double getMotorAvgPosition() {
//            return (double) (Math.abs(motorFrontLeft.getCurrentPosition()) + Math.abs(motorFrontRight.getCurrentPosition()) + Math.abs(motorBackLeft.getCurrentPosition()) + Math.abs(motorBackRight.getCurrentPosition())) / 4.0;
//        }
//    }
//
//    private DcMotor motorFrontRight = null;
//    private DcMotor motorBackLeft = null;
//    private DcMotor motorFrontLeft = null;
//    private DcMotor motorBackRight = null;
//    private DcMotor arm = null;
//    private DcMotor elbow = null;
//    private Servo clawWrist = null;
//    private Servo servoClaw = null;
//    private Servo servoRailgun = null;
//    private Servo wrist = null;
//    private double powerLeft = 0;
//    private double powerRight = 0;
//    private int elbowPosition = 0;
//    private int targetElbowPosition = 0;
//    private double wristPosition = 0;
//    private double targetWristPosition = 0;
//    private double max = 0.6;
//    private double min = .4;
//    private int armPosition = 0;
//    private int targetArmPos = 0;
//    private int maxArm = Integer.MAX_VALUE;
//
//    public void arm(){
//        telemetry.addData("arm position", arm.getCurrentPosition());
//
//    }
//    public void claws() {
//        telemetry.addData("santa claws claws", servoClaw.getPosition());
//        if (gamepad1.left_bumper) {
//            servoClaw.setPosition(servoNormalize(1935));
//        }
//        else{
//
//        }
//        if (gamepad1.right_bumper) {
//            servoClaw.setPosition(servoNormalize(1665));
//        }
//        else{
//
//        }
//    }
//
//    public void clawWrist() {
//        telemetry.addData("Claw wrist position:", clawWrist.getPosition());
//        if (gamepad1.dpad_right) {
//            clawWrist.setPosition(servoNormalize(2105));
//        }
//
//        if (gamepad1.dpad_left) {
//            clawWrist.setPosition(servoNormalize(1221));
//        }
//    }
//
//}
//    public void setServoRailgun() {
//        telemetry.addData("servo position", servoRailgun.getPosition());
//        if (gamepad1.dpad_up) {
//            servoRailgun.setPosition(servoNormalize(1080));
//        }
//
//        if (gamepad1.dpad_down) {
//            servoRailgun.setPosition(servoNormalize(1825))