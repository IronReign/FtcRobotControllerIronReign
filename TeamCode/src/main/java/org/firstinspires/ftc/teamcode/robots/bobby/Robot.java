//4 feet 5 in

package org.firstinspires.ftc.teamcode.robots.bobby;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.LinkedHashMap;
import java.util.Map;

import com.acmerobotics.dashboard.FtcDashboard;
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
    int startServoPosition = 1400;
    int pushUp = 1070;
    Servo flywheelPusher;

    // VISION VARS
    OpenCvWebcam webcam;
    BallPipeline pipeline;


    // ODOMETRY VARS.
    public static double TICKS_PER_REV = 537.7;   // REV HD Hex 20:1
    public static double WHEEL_RADIUS = 1.476;    // inches (75mm / 2)
    public static double CHAIN_RATIO = 20.0/15;   // wheel not directly on motor

    // remeber to mesure
    public static double TRACK_WIDTH = 14.5;   // inches (left ↔ right wheel distance)
    public static double WHEEL_BASE  = 16.5;   // inches (front ↔ back wheel distance)
    private double lastHeading = 0.0;



    // ROBOT POSITION VARS
//    public double x = -51.44;
//    public double y =51.44;

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
    double headingOffset = 0.0;   // ADDED

    // ROBOT CONSTRUCTOR
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
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

        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelPusher.setPosition(servoNormalize(startServoPosition));


        int cameraMonitorViewId =
                hardwareMap.appContext
                        .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                0
        );

        pipeline = new BallPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam,0);
            }

            @Override
            public void onError(int errorCode) {}
        });


        // IMU INITIALIZATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;

        imu.initialize(params);

       //  -red aliance
        headingOffset = getRawHeading() - Math.toRadians(0);

        // - blue aliance
//        headingOffset = getRawHeading() - Math.toRadians(45);

//        resetHeading();
        heading = getHeading(); // initial heading in field coordinates
        lastHeading = heading;
        // -------------------------------


        lastLF = leftFront.getCurrentPosition();
        lastRF = rightFront.getCurrentPosition();
        lastLB = leftBack.getCurrentPosition();
        lastRB = rightBack.getCurrentPosition();
    }

//public void updatePose() {
//    int lf = leftFront.getCurrentPosition();
//    int rf = rightFront.getCurrentPosition();
//    int lb = leftBack.getCurrentPosition();
//    int rb = rightBack.getCurrentPosition();
//
//    int dLF = lf - lastLF;
//    int dRF = rf - lastRF;
//    int dLB = lb - lastLB;
//    int dRB = rb - lastRB;
//
//    lastLF = lf;
//    lastRF = rf;
//    lastLB = lb;
//    lastRB = rb;
//
//    double dlf = ticksToInches(dLF);
//    double drf = ticksToInches(dRF);
//    double dlb = ticksToInches(dLB);
//    double drb = ticksToInches(dRB);
//
//    double dxRobot = (dlf + drf + dlb + drb) / 4.0;
//    double dyRobot = (-dlf + drf + dlb - drb) / 4.0;
//
//    heading = getHeading();
//
//    double cos = Math.cos(heading);
//    double sin = Math.sin(heading);
//
//    // SWAPPED: dyRobot affects field X, dxRobot affects field Y
//    x += dyRobot * cos + dxRobot * sin;
//    y += dyRobot * sin - dxRobot * cos;
//}


    public void updatePose()
    {

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


        // Update heading to separate rotational and strafe motion
        heading = getHeading();
//        double dTheta = heading - lastHeading;
//
//        while(dTheta > Math.PI) dTheta -= 2 * Math.PI;
//        while(dTheta < -Math.PI) dTheta += 2 * Math.PI;
//
//        lastHeading = heading;
//
//        // Calculate ticks lost to rotational motion
//        double rotationRadius =( TRACK_WIDTH + WHEEL_BASE) / 2 ;
//        double rotationDist = dTheta * rotationRadius;
//
//        // Remove from each wheel
//        dlf -= rotationDist;
//        drf += rotationDist;
//        dlb -= rotationDist;
//        drb += rotationDist;

        double dTheta = heading - lastHeading;

        while (dTheta > Math.PI) dTheta -= 2 * Math.PI;
        while (dTheta < -Math.PI) dTheta += 2 * Math.PI;

        lastHeading = heading;

        double rotationRadius = (TRACK_WIDTH + WHEEL_BASE) / 2.0;
        double rotationDist = dTheta * rotationRadius / 2.0;

        dlf -= rotationDist;
        drf += rotationDist;
        dlb -= rotationDist;
        drb += rotationDist;




        // Calculate change in robots position
        double dxR = (dlf + drf + dlb + drb) / 4.0;
        double dyR = ( -dlf + drf + dlb - drb)/ 4.0;

        // Calculate
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Update robots position
        x += dyR * cos + dxR * sin;
        y += dyR * sin - dxR * cos;

        // Try This if axes are flipped
    //    x += dxRobot * cos - dyRobot * sin;
    //    y += dxRobot * sin + dyRobot * cos;
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

    public void setFlywheelPower(double x) {
        flywheelL.setPower(x);
        flywheelR.setPower(x);
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