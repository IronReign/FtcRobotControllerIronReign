package org.firstinspires.ftc.teamcode.robots.goldenduck.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

public class AutoDrive {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackRight = null;
    private double powerLeft = 0;
    private double powerRight = 0;
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
        //imu
        BNO055IMU imu;
        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;    // State used for updating telemetry
        public double heading;

        // power input for each respective wheel
        private static final float DEADZONE = .1f;
        double robotSpeed = 1;
        public AutoDrive (Telemetry telemetry, HardwareMap hardwareMap) {
            this.telemetry = telemetry;
            this.hardwareMap = hardwareMap;
        }
        public void mecanumAuto(double forward, double strafe, double turn){
            forward = forward;
            turn = turn;
            double r = Math.hypot(strafe, forward);
            double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
            double rightX = turn;
            powerFrontLeft = r * Math.cos(robotAngle) - rightX;
            powerFrontRight = r * Math.sin(robotAngle) + rightX;
            powerBackLeft = r * Math.sin(robotAngle) - rightX;
            powerBackRight = r * Math.cos(robotAngle) + rightX;
            motorFrontLeft.setPower(powerFrontLeft*robotSpeed);
            motorFrontRight.setPower(powerFrontRight*robotSpeed);
            motorBackLeft.setPower(powerBackLeft*robotSpeed);
            motorBackRight.setPower(powerBackRight*robotSpeed);
        }
        public void update(){

            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            heading = angles.firstAngle;

            telemetry.addData("Back Right Position \t", motorBackRight.getCurrentPosition());
            telemetry.addData("Back Left Position \t", motorBackLeft.getCurrentPosition());
            telemetry.addData("Front Right Position \t", motorFrontRight.getCurrentPosition());
            telemetry.addData("Front Left Position \t", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Average Motor Position \t", getMotorAvgPosition());
            telemetry.addData("Heading \t", formatAngle(angles.angleUnit, heading));

        }
        public void driveInit(){
            motorFrontLeft = this.hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
            motorBackLeft = this.hardwareMap.get(DcMotorEx.class, "motorBackLeft");
            motorFrontRight = this.hardwareMap.get(DcMotorEx.class, "motorFrontRight");
            motorBackRight = this.hardwareMap.get(DcMotorEx.class, "motorBackRight");
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

        }

        public void turnOffMotors(){
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontRight.setPower(0);

        }
        public void resetMotors() {
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontRight.setPower(0);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
            this.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
            motorFrontLeft.setPower(1);
            motorBackLeft.setPower(1);
            motorBackRight.setPower(1);
            motorFrontRight.setPower(1);
            mecanumAuto(0, 0, 0);
        }

        public double getMotorAvgPosition(){return (double)(Math.abs(motorFrontLeft.getCurrentPosition())+Math.abs(motorFrontRight.getCurrentPosition())+Math.abs(motorBackLeft.getCurrentPosition())+Math.abs(motorBackRight.getCurrentPosition()))/3.0;}

        String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees(double degrees){
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }

        public double getHeading(){
            return heading;
        }
    }
