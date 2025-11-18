package org.firstinspires.ftc.teamcode.robots.lebot;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.lebot.Robot;

@TeleOp (name = "test")
public class opMode extends OpMode {
    Robot robot = new Robot(hardwareMap, gamepad1);
    tankDrive drivetrain = new tankDrive();
    StickyGamepad g1;
    private BNO055IMU imu;
    double throttle, spin;

    boolean turning = false;
    double refrenceAngle = Math.toRadians(90);
    double integralSum = 0;
    private double lastError = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        robot.init(hardwareMap);
        g1 = new StickyGamepad(gamepad1);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    @Override
    public void loop() {
        g1.update();
        handleJoysticks(gamepad1);
    }

    //TODO: direction of joystick correlates to turning angle
    public void handleJoysticks(Gamepad gamepad) {
        if (g1.a && !turning) {
            turning = true;
            integralSum = 0;
            lastError = 0;
        }

        if(turning){

            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            double error = angleWrap(refrenceAngle - currentAngle);

            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();
            lastError = error;
            timer.reset();

            double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

            drivetrain.power(output);

            if (Math.abs(error) < Math.toRadians(2)){
                drivetrain.power(0);
                turning = false;
            }

        } else {
            throttle = -gamepad1.left_stick_y;
            spin = -gamepad1.left_stick_x;
            drivetrain.drive(throttle, spin);
        }

        if(g1.b) {
            robot.fireBall();
        }
    }

    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
