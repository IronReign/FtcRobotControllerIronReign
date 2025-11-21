package org.firstinspires.ftc.teamcode.robots.lebot;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.lebot.Robot;

import java.util.Map;

@TeleOp (name = "test")
public class opMode extends OpMode {
    Robot robot = new Robot(hardwareMap, gamepad1);
    //tankDrive drivetrain = new tankDrive();
    StickyGamepad g1;
    //private BNO055IMU imu;
    double throttle, spin;
    boolean damp=false;

//    boolean turning = false;
//    double refrenceAngle = Math.toRadians(90);
//    double integralSum = 0;
//    private double lastError = 0;
//    double Kp = PIDConstants.Kp;
//    double Ki = PIDConstants.Ki;
//    double Kd = PIDConstants.Kd;
//
//    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        //drivetrain.init(hardwareMap);
        robot.init(hardwareMap);
        g1 = new StickyGamepad(gamepad1);
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //imu.initialize(parameters);

    }

    @Override
    public void loop() {
        g1.update();
        handleJoysticks(gamepad1);
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());

//        if(g1.b) {
//            robot.fireBall();
//        }


    }

    @Override
    public void init_loop(){
        g1=new StickyGamepad(gamepad1);
        g1.update();
        if(g1.b){
            robot.setRedALliance(true);
            robot.switchPipeline(1);
        }
        if(g1.x){
            robot.setRedALliance(false);
            robot.switchPipeline(0);
        }
        robot.update(new Canvas());
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }

    //TODO: direction of joystick correlates to turning angle
    public void handleJoysticks(Gamepad gamepad) {
        throttle = -gamepad1.left_stick_y;
        if(g1.y) {
            robot.shooting = !robot.shooting;
        }
        double dampen;

        if(g1.right_bumper){
            damp=!damp;
        }
        if(damp){
            dampen=.2;
        }else{
            dampen=1;
        }
        if(g1.a){
            if(Math.abs(robot.getCurrentIMU()-robot.getRefrenceAngle())>Math.toRadians(1.5)){
                robot.setTurning(true);
            }else{
                robot.setTurning(false);
            }
//            if(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle-refrenceAngle)>Math.toRadians(1.5)) {
//                turning=true;
//            }else{
//                turning=false;
//            }
        }
        if(g1.x){
            if(robot.tx()){
            //if(robot.tx() && Math.abs(robot.gettx())>Math.toRadians(1.5)){
                robot.setTurningT(true);
            }else{
                robot.setTurningT(false);
            }
        }
        if(robot.getTurningT()){
            robot.turnToTag();
            //robot.turnItShoot();
            robot.setDrivetrain(throttle, 0);
        }
//        if (!turning) {
//            calculateTurn();
//        }

        else if (robot.getTurning()) {
            robot.turnIt();
            robot.setDrivetrain(throttle, 0);
            return;
        }else{
            robot.setDrivetrain(throttle, (dampen)*-gamepad1.left_stick_x);
        }


    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        telemetry.addLine(telemetryName);
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
        }
        telemetry.addLine();
    }

//    public double angleWrap(double radians){
//        while(radians > Math.PI){
//            radians -= 2 * Math.PI;
//        }
//        while(radians < -Math.PI){
//            radians += 2 * Math.PI;
//        }
//        return radians;
//    }
//
//    public void calculateTurn(){
//        double rightx = gamepad1.right_stick_x;
//        double righty = gamepad1.right_stick_y;
//
//        if (Math.hypot(rightx, righty)>0.15) {
//            refrenceAngle = Math.atan2(-rightx, -righty);
//            turning = true;
//            integralSum = 0;
//            lastError = 0;
//            timer.reset();
//        }
//    }
//
//    public void turnIt(){
//
//        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//        double error = angleWrap(refrenceAngle - currentAngle);
//
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//        timer.reset();
//
//        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
//
//        drivetrain.power(output);
//
//        if (Math.abs(error) < Math.toRadians(1.5)){
//            drivetrain.power(0);
//            turning = false;
//        }
//    }
}