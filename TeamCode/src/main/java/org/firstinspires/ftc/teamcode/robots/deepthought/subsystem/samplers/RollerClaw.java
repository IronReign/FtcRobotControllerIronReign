package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.Map;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.*;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;


@TeleOp

public class RollerClaw extends OpMode {
    public CRServo CRSOne;
    public CRServo CRSTwo;
    public DcMotor motor;
    public NormalizedColorSensor colorSensor;


    boolean INTAKE;
    boolean EJECT;
    boolean colSensorStat;


    public Constants.Alliance alliance = Constants.Alliance.RED;
    String allianceCol = "RED";
    String appoAllianceCol = "BLUE";

    public void init() {
        CRSOne = this.hardwareMap.get(CRServo.class, "CRSOne");
        CRSTwo = this.hardwareMap.get(CRServo.class, "CRSTwo");
        colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        INTAKE = false;
        EJECT = false;
        colSensorStat = false;
    }
// 90,60,120,150 - no sample
//    200-215 - blue
    // 0-15 - red
    // 50-59 - no sample
    public void loop() {
        if (INTAKE) {
            if(!sampleDetected()) {
                CRSOne.setPower(-1);
                CRSTwo.setPower(1);
                colSensorStat = true;
            }
            else if(apposingSample()) {
                INTAKE = false;
                EJECT = true;
            }
            else{
                CRSOne.setPower(0);
                CRSTwo.setPower(0);
            }
        }
        double ejectTimer;
        ejectTimer = futureTime(.5);
        if (EJECT) {

                if (sampleDetected()) {
                    CRSOne.setPower(1);
                    CRSTwo.setPower(-1);
                    colSensorStat = true;

//                     if(!sampleDetected()){
//                        CRSOne.setPower(-1);
//                        CRSTwo.setPower(1);
//                        colSensorStat = true;
//                    }
                }
                else if(!sampleDetected()) {
                    if(CRSOne.getPower() == 1 && CRSTwo.getPower() == -1){
                        EJECT = false;
                        INTAKE = true;
                        colSensorStat = true;
                    }
                    else{
                        EJECT = false;
                        INTAKE = false;
                        colSensorStat = false;
                    }
                }
//            telemetry.addData("ejectTimer", ejectTimer);
            }

        if (!INTAKE && !EJECT) {
            CRSOne.setPower(0);
            CRSTwo.setPower(0);
            colSensorStat = false;
        }




        if (gamepad1.y) {
            INTAKE = true;
            EJECT = false;
        }
        if (gamepad1.b) {
            EJECT = true;
            INTAKE = false;
        }
        if (gamepad1.a) {
            INTAKE = false;
            EJECT = false;
        }
        if(gamepad1.x){
            if(allianceCol.equals("RED")){
                alliance = Constants.Alliance.BLUE;
                allianceCol = "BLUE";
                appoAllianceCol = "RED";
            }
            else{
                alliance = Constants.Alliance.RED;
                allianceCol = "RED";
                appoAllianceCol = "BLUE";
            }
        }

//        if(INTAKE) {
////            if (stopOnSample()) {
////               // colSensorStat = false;
////                INTAKE = false;
////                EJECT = false;
////            }
////        if(noSample()){
////            EJECT = false;
////            //INTAKE = true;
////        }
////            if (apposingSample()) {
////                //colSensorStat = false;
////                INTAKE = false;
////                EJECT = true;
////            }
//        }



//        if (gamepad1.a) {
//            INTAKE = false;
//            EJECT = false;
//            colSensorStat = false;
//        }
//        if(stopOnSample()){
//            colSensorStat = false;
//            INTAKE = false;
//            //OUTAKE = false;
//        }




        telemetry.update();
        telemetry.addData("in", INTAKE);
        telemetry.addData("out", EJECT);
        telemetry.addData("power1", CRSOne.getPower());
        telemetry.addData("power2", CRSTwo.getPower());
        telemetry.addData("colorSensorStatus", colSensorStat);
        telemetry.addData("Sample", colSensorView());
        telemetry.addData("colorSensorHSV", getHSV()[0]);
        telemetry.addData("Alliance", allianceCol);
        telemetry.addData("ejectTimer", ejectTimer);


    }

    public boolean stopOnSample(){
        if(colSensorView().equals(allianceCol) || colSensorView().equals("neutral")){

            return true;
        }
        return false;
    }

    public boolean apposingSample(){
        if(colSensorView().equals(appoAllianceCol)){
            return true;
        }
        return false;
    }
    public boolean sampleDetected(){
        if(colSensorView().equals("noSample")){
            return false;
        }
        return true;
    }

    public float[] getHSV(){
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(),hsv);
        return hsv;
    }
//
    public String colSensorView(){
        double hue = getHSV()[0];
        if(hue < 59 && hue > 40){
            return "neutral";
        }
        else if(hue < 15 && hue > 1){
            return "RED";
        }
        else if(hue < 215 && hue > 200){
            return "BLUE";
        }
        else{
            return "noSample";
        }
   }
    // 90,60,120,150 - no sample
//    200-215 - blue
    // 0-15 - red
    // 50-59 - no sample





}
