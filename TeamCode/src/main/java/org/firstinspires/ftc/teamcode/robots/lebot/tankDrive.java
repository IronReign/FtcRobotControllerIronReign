package org.firstinspires.ftc.teamcode.robots.lebot;


import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.old.Sensors;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class tankDrive {

    private DcMotor leftFront, rightFront;
    private DcMotor leftBack, rightBack;
    private BNO055IMU imu;
    public boolean imuTurnDone=false;
    private double targetHeading, targetVelocity = 0;
    public static PIDController headingPID;
    static double Kp = PIDConstants.Kp;
    static double Ki = PIDConstants.Ki;
    static double Kd = PIDConstants.Kd;
    public static PIDCoefficients HEADING_PID_PWR = new PIDCoefficients(Kp, Ki, Kd);
    public static double HEADING_PID_TOLERANCE = 3.5; //this is a percentage of the input range .063 of 2PI is 1 degree
    private double PIDCorrection, PIDError;

    public void init(HardwareMap hardwareMap){

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        headingPID = new PIDController(HEADING_PID_PWR);
        headingPID.setInputRange(0, 360);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();
        /*rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);*/
    }

    public void drive(double throttle, double spin) {
        double leftPower = throttle + spin;
        double rightPower = throttle - spin;
        double largest = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if(largest > 1.0) {
            leftPower /= largest;
            rightPower /= largest;
        }

        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);

        /*leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);*/
    }

    public void power(double output){
        leftFront.setPower(output);
        rightFront.setPower(-output);

        /*leftBack.setPower(output);
        rightBack.setPower(-output);*/

    }

    public boolean turnUntilDegreesIMU(double turnAngle, double maxSpeed) {
        Sensors.driveIMUEnabled = true;
        targetHeading = wrapAngle(turnAngle);
        headingPID.setPID(HEADING_PID_PWR);
        headingPID.setInput(wrapAngle(Robot.sensors.driveIMUYaw)); //seems to be heading from roadrunner
        headingPID.setSetpoint(targetHeading);
        headingPID.setOutputRange(-maxSpeed, maxSpeed);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        double correction = headingPID.performPID();
        PIDCorrection = correction;
        PIDError = headingPID.getError();
        if (headingPID.onTarget()) {
            //turn meets accuracy target
            //todo is this a good time to update getPose() heading from imu?
            //setPose(new Pose2d(getPose().position, Math.toRadians(Robot.sensors.driveIMUYaw)));
            //stop
            Sensors.driveIMUEnabled = false;
            power(0);
            return imuTurnDone = true;
        } else {
            headingPID.enable();
            headingPID.setSetpoint(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            double power = headingPID.performPID();
            power(power);
            //setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), correction));
            return imuTurnDone = false;
        }
    }

    public void turnLeft(double x){
        rightFront.setPower(x*.3);
    }
    public void turnRight(double x){
        leftFront.setPower(x*.3);
    }
}
