//4 feet 5 in

package org.firstinspires.ftc.teamcode.robots.bobby;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.LinkedHashMap;
import java.util.Map;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Config(value= "bobby")
public class Robot implements Subsystem {
    HardwareMap hardwareMap;
    Gamepad gamepad1;

    // MOTOR VARS
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotorEx rightFront;
    DcMotorEx leftFront;
    DcMotorEx flywheelL;
    DcMotorEx flywheelR;
    DcMotorEx intake;


    // SERVO VARS
    int startServoPosition = 1390;
    int pushUp = 1070;
    Servo flywheelPusher;

    // VISION VARS
    OpenCvWebcam webcam;
    MasterVisionPipeline pipeline;


    // ODOMETRY VARS.
    public static double TICKS_PER_REV = 537.7;   // REV HD Hex 20:1
    public static double WHEEL_RADIUS = 1.476;    // inches (75mm / 2)
    public static double CHAIN_RATIO = 20.0/15;   // wheel not directly on motor


    public double x = 0.0;
    public double y = 0.0;
    public double heading = 0.0;  //radians


    // ENCODER STATE
    private int lastLF = 0;
    private int lastRF = 0;
    private int lastLB = 0;
    private int lastRB = 0;

    //IMU DECLARATION
    BNO055IMU imu;                // ADDED
    double headingOffset;   // ADDED

    double angleStart;

    // ---------- CAMERA / APRILTAG PARAMS ----------
    public static final double TAG_SIZE = 0.166; // meters (6.5 inches)

    public static final double FX = 578.272;
    public static final double FY = 578.272;
    public static final double CX = 402.145;
    public static final double CY = 221.506;

    // ROBOT CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, double angle) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        angleStart = angle;
    }

    @Override
    public void update(Canvas fieldOverlay)
    {
        updatePose();
        if(gamepad1 !=null)
        {
            mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            slowTurn(gamepad1.dpad_right,gamepad1.dpad_left);
        }

    }


    //DRIVETRAIN
    public void mecanumDrive(double forward, double strafe, double turn) {

        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        // Normalize measurements for proportional power
        double biggest = Math.max(  1.0 ,   Math.max(    Math.abs(lf)   ,  Math.max(  Math.abs(rf) , Math.max( Math.abs(lb) , Math.abs(rb)  )  )   )   );
        biggest = 1;
        leftBack.setPower( lb / biggest);
        rightBack.setPower( rb / biggest);
        leftFront.setPower( lf / biggest);
        rightFront.setPower( rf/ biggest);
    }

    public void slowTurn(boolean right, boolean left) {
      if(right) mecanumDrive(0,0,0.2);
      else if(left) mecanumDrive(0,0,-0.2);
    }

    // INITIALIZATION
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        flywheelL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        flywheelPusher = hardwareMap.get(Servo.class, "flywheelPusher");


        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

//        leftBack.setDirection(DcMotor.Direction.REVERSE);
//        leftFront.setDirection(DcMotor.Direction.REVERSE);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);


        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);

        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flywheelL.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(30, 0, 2, 11)
        );

        flywheelR.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(30, 0, 2, 11)
        );

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelPusher.setPosition(servoNormalize(startServoPosition));


//        int cameraMonitorViewId =
//                hardwareMap.appContext
//                        .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"),
//                0
//        );
//
//        pipeline = new MasterVisionPipeline(
//                TAG_SIZE,
//                FX,FY,
//                CX, CY
//        );
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(webcam,0);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });


        // IMU INITIALIZATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;

        imu.initialize(params);

        resetHeading();
        headingOffset -= Math.toRadians(angleStart);
        heading = getHeading(); // initial heading in field coordinates

        // -------------------------------
        lastLF = leftFront.getCurrentPosition();
        lastRF = rightFront.getCurrentPosition();
        lastLB = leftBack.getCurrentPosition();
        lastRB = rightBack.getCurrentPosition();
    }


    public void updatePose()  {

        // Read in encoder values
        int lf = leftFront.getCurrentPosition();
        int rf = rightFront.getCurrentPosition();
        int lb = leftBack.getCurrentPosition();
        int rb = rightBack.getCurrentPosition();


        // Find change in each value
        int dLF = lf - lastLF;
        int dRF = rf - lastRF;
        int dLB = lb - lastLB;
        int dRB = rb - lastRB;


        // Update lastPosition
        lastLF = lf;
        lastRF = rf;
        lastLB = lb;
        lastRB = rb;


        // Convert motor ticks into inches
        double dlf = ticksToInches(dLF);
        double drf = ticksToInches(dRF);
        double dlb = ticksToInches(dLB);
        double drb = ticksToInches(dRB);

        heading = getHeading();

        // Calculate change in robots position
        double dxR = (dlf + drf + dlb + drb) / 4.0;
        double dyR = ( -dlf + drf + dlb - drb)/ 4.0;

        // Calculate
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Update robots position
        x += dyR * cos + dxR * sin;
        y += dyR * sin - dxR * cos;
    }

    public void setTurnPower(double turn) {
        mecanumDrive(0,0,turn) ;
    }

    public double getRawHeading()
    {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    public double getHeading()
    {
        double h =  getRawHeading() - headingOffset;

        while (h > Math.PI) h -= 2 * Math.PI;
        while (h < -Math.PI) h += 2 * Math.PI;

        return h;
    }

    public void resetHeading() {
        headingOffset = getRawHeading();
        heading = 0.0;
    }

    private double ticksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * CHAIN_RATIO * ticks / TICKS_PER_REV;
    }

    public void setFlywheelVelocity(double rpm) {
        double FLYWHEEL_TICKS_PER_REV = 1120.0;
        double ticksPerSecond = (rpm * FLYWHEEL_TICKS_PER_REV) / 60.0;

        flywheelL.setVelocity(ticksPerSecond);
        flywheelR.setVelocity(ticksPerSecond);
    }
    public void setIntakePower(double x) {
        intake.setPower(x);
    }
    public void setPusherUp(){
        flywheelPusher.setPosition(servoNormalize(pushUp));
    }
    public void setPusherDown(){
        flywheelPusher.setPosition(servoNormalize(startServoPosition));
    }


    @Override
    public void stop() {
    }
    @Override
    public void resetStates() {
    }
    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();
        telemetry.put("x (inches)", x);
        telemetry.put("y (inches)", y);
        telemetry.put("heading (rad)", heading);
        telemetry.put("LF encoder", leftFront.getCurrentPosition());
        telemetry.put("Flywheel L velocity", flywheelL.getVelocity()*60/ 1120);
        telemetry.put("Flywheel R velocity", flywheelR.getVelocity()*60/1120);

        return telemetry;
    }
    @Override
    public String getTelemetryName(){
        return "bobby robot";
    }
    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}