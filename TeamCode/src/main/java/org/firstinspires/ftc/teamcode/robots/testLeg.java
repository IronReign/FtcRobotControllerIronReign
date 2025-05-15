package org.firstinspires.ftc.teamcode.robots;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.StickyGamepad;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import java.util.ArrayList;

@TeleOp
public class testLeg extends OpMode{
    int CALIBRATE_POSITION = Integer.MAX_VALUE;
    FtcDashboard dashboard;
    DcMotorEx muscleOne;
    private DcMotorEx muscleOneEncoder;
    private int pos;
    private int contract;
    private int extend;
    private int middle;
    private int position;
    private long startTime;
    StickyGamepad g1;
    @Override
    public void init(){
        startTime = System.currentTimeMillis();
        dashboard = FtcDashboard.getInstance();
         muscleOne = hardwareMap.get(DcMotorEx.class, "muscleOne");
         pos = muscleOne.getCurrentPosition();
         g1=new StickyGamepad(gamepad1);
        muscleOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        muscleOne.setTargetPosition(muscleOne.getCurrentPosition());
        muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


    }

    @Override
    public void start() {
        //muscleOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //muscleOne.setTargetPosition(muscleOne.getCurrentPosition());
        //muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //extend = getExtended();
        //contract = getContracted();
        //middle = Math.abs(getExtended() - getContracted());

        //long start = System.currentTimeMillis();
        //while(System.currentTimeMillis() - start < 1000) {
        //}
        contract = getContracted();
        position  = 0;
        myWait(1);
        extend = getExtended();
        middle = Math.abs(getExtended() + getContracted()) / 2;
    }

    @Override
    public void init_loop() {
        updateTelemetry();
    }

    @Override
    public void loop(){

        //int tuck = -394;
        //int strait = 210;
        //int mid = -40;
        //int zero = 0;
        int ticks = muscleOne.getCurrentPosition();
        pos = muscleOne.getCurrentPosition();
        // 304 is fully extended
        // about mid is -40
        //down is -394
        if (gamepad1.a) {
            muscleOne.setTargetPosition(contract);
            muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            muscleOne.setPower(1);
        }
        if (gamepad1.y) {
            muscleOne.setTargetPosition(extend);
            muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            muscleOne.setPower(-1);
        }

        if (gamepad1.b) {
            muscleOne.setTargetPosition(middle);
            muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            muscleOne.setPower(-1);
        }

        if(gamepad1.x){
            jump();
        }
        if(gamepad1.right_trigger > 0.3){
            ticks += 45;
            muscleOne.setTargetPosition(ticks);
            muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            muscleOne.setPower(1);
            position = ticks;
        }

        if(gamepad1.left_trigger > 0.3){
            ticks -= 45;
            muscleOne.setTargetPosition(ticks);
            muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            muscleOne.setPower(-1);
            position = ticks;
        }

        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("position", position);
        telemetry.addData("extended postion: ",extend);
        telemetry.addData("contracted postion: ",contract);
        telemetry.addData("middle postion: ",middle);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Chassis Heading", muscleOneEncoder);

        dashboard.sendTelemetryPacket(packet);

        telemetry.update();
    }


    public int getContracted() {


        int tempPos = position;
        boolean go = true;

        while(go){
            tempPos -= 45;
            muscleOne.setPower(-0.4);
            muscleOne.setTargetPosition(tempPos);
            muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            if(muscleOne.getCurrent(CurrentUnit.AMPS) > 5){
                go = false;
            }


        }

        return tempPos;
    }

    public int getExtended() {
        int tempPos = position;
        boolean go = true;

        while(go){
            tempPos += 45;
            muscleOne.setTargetPosition(tempPos);
            muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            muscleOne.setPower(1);

            if(muscleOne.getCurrent(CurrentUnit.AMPS) > 0.85){
                go = false;
            }
        }

        return tempPos;
    }

    public void jump() {
        muscleOne.setTargetPosition(contract);
        muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        muscleOne.setPower(1);
        myWait(0.05);
        muscleOne.setTargetPosition(extend);
        muscleOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        muscleOne.setPower(-1);

    }

    //give time in seconds
    public void myWait(double givenWait){
        long startCount = System.currentTimeMillis();
        while(System.currentTimeMillis() - startCount < givenWait * 1000){

        }
        //return true;
    }
}
