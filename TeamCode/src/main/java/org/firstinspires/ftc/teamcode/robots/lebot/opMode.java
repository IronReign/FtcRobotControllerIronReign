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
    //private FtcDashboard dashboard;
    //tankDrive drivetrain = new tankDrive();
    StickyGamepad g1;
    //private BNO055IMU imu;
    double throttle, spin;
    boolean damp=false;
    boolean suck=false;
    boolean shoot=false;
    boolean pushUp=false;
    boolean eject=false;
    boolean adjust = false;



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
        robot = new Robot(hardwareMap, gamepad1);
        robot.init();
        robot.setPaddleDown();
        //robot.setServoUp();
        g1 = new StickyGamepad(gamepad1);
        //TELEMETRY SETUP
        telemetry.setMsTransmissionInterval(250);

    }

    @Override
    public void loop() {
        g1.update();
        handleJoysticks(gamepad1);
        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
        robot.updateDistance();
        robot.update(new Canvas());
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

    public void handleJoysticks(Gamepad gamepad) {
        throttle = -gamepad1.left_stick_y;
//        if(g1.b) {
//            shoot=!shoot;
//        }
        if(g1.b){
            robot.setShoot(false);
            robot.shoot(false);
            suck=false;
        }
        if(g1.dpad_up){
            robot.setPaddleClear();
            //pushUp=!pushUp;
        }
        if(g1.dpad_down){
            robot.setPaddleDown();
        }
        double dampen;
        if(Math.abs(gamepad1.right_stick_x) >.3 ||Math.abs(gamepad1.right_stick_y) >.3||Math.abs(gamepad1.left_stick_x) >.3 ||Math.abs(gamepad1.left_stick_y) >.3 ){
            robot.setTurningT(false);
            robot.setTurning(false);
        }

        if(g1.right_bumper){
            robot.setTurning(true);
        }

        if(g1.left_bumper){
            damp=!damp;
        }
        if(damp){
            dampen=.15;
        }else{
            dampen=.67;
        }
        if(g1.dpad_left){
            suck=!suck;
        }

        if(g1.dpad_right){
            //pushUp=!pushUp;
            eject=!eject;
        }
//        if(pushUp){
//            robot.openChannel();
//        }else{
//            robot.closedChannel();
//        }
//        if(shoot){
//            robot.shoot(true);
//        }else{
//            robot.shoot(false);
//        }
        if(!robot.getShoot()){
            if(suck){
                robot.intakeOn();
            }else if(eject) {
                robot.setIntakeSpeed(.2);
            }else{
                robot.intakeOff();
            }
        }

        if(g1.a){
            robot.resetShootIndex();
            robot.setShoot(true);
        }
        if(g1.y){
            if(robot.tx()){
                robot.setTurningT(true);
            }

        }
//        if(g1.x){
//            //robot.setShoot(898);
//            shoot=!shoot;
//            //robot.setShoot(1);
//        }
//        if(shoot){
//            robot.setShoot(898);
//        }else{
//            robot.setShoot(0);
//        }


        if(!robot.getTurningT()){
            //if(throttle>0){
                robot.setDrivetrain(throttle, (dampen)*-gamepad1.right_stick_x);
            //}

        }


//        if (robot.getTurning()) {
//            robot.turnIt();
//            robot.setDrivetrain(throttle, 0);
//            return;
//        }else{
//        if(!damp){

//            robot.setTurningL(false);
//            robot.setTurningR(false);
        }
//        else{
//            if(gamepad1.right_stick_x>0){
//                robot.setTurningR(true);
//            }else{
//                robot.setTurningL(true);
//            }
//        }

        //}

//        if(g1.b) {
//            //robot.resetIndex();
//            index=0;
//            fireBall();
//        }
//
//        if(g1.dpad_right) {
//            index++;
//        }
//
//        if (g1.dpad_left){
//            index--;
//        }


   // }
//    int index=0;
//    private long timer2;
//    public void fireBall() {
//        //countBalls();
//
//
////        ElapsedTime time = new ElapsedTime();
////        time.reset();
//        switch (index) {
//            case 0:
//                robot.closedChannel();
//                robot.intakeOn();
////                time.reset();
////                while(time.milliseconds()<2000){
////
////                }
////                index=index+1;
//                if (robot.channelDistFull) {
//                    index++;
//                }
//                break;
//
//            case 1:
//                robot.intakeOff();
//                robot.shoot(true);
//                timer2 = futureTime(5);
//                if (isPast(timer2)) {
//                    index++;
//                }
//
//                break;
//
//            case 2:
//                if (isPast(timer2)) {
//                    robot.feedPaddle();
//                    index++;
//                }
//                break;
//
//            case 3:
//                robot.openChannel();
//                robot.conveyor.setPower(0);
//                timer2 = futureTime(3);
//                index++;
//                break;
//
//            case 4:
//                if (isPast(timer2)) {
//                    robot.closedChannel();
////                    if(numBalls == 0){
////                        shooter.setPower(0);
////                        shootingState = shootingState.RESET;
////                    } else {
////                        shootingState = shootingState.SPIN;
////                    }
//
//                }
//                break;
//        }
    //}

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        telemetry.addLine(telemetryName);
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
        }
        telemetry.addLine();
    }
}
