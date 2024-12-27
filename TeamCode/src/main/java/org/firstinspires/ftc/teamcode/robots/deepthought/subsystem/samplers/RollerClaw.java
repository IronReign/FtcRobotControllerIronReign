package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import java.util.Map;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import java.util.*;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;





//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
@Config()
public class RollerClaw {
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Telemetry telemetry;
    CRServo servo1;
    CRServo servo2;
    boolean INTAKE;
    boolean OUTAKE;

    public void init() {
        servo1 = hardwareMap.get(CRServo.class,"servo1");
        servo2 = hardwareMap.get(CRServo.class,"servo2");
        INTAKE = false;
        OUTAKE = false;

    }

    public void loop() {
        if(INTAKE) {
            servo1.setPower(-1);
            servo2.setPower(1);
        }
        else if(OUTAKE) {
            servo1.setPower(1);
            servo2.setPower(-1);
        }
        else
        {
            servo1.setPower(0);
            servo2.setPower(0);
        }



        if(gamepad1.a) {
            INTAKE = false;
            OUTAKE = false;
        }
        if(gamepad1.b){
            INTAKE = true;
            OUTAKE = false;
        }
        if(gamepad1.x){
            OUTAKE = true;
            INTAKE = false;
        }

        telemetry.addData("in", INTAKE);
        telemetry.addData("out", OUTAKE);

    }
}
