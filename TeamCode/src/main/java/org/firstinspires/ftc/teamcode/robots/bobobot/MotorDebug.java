package org.firstinspires.ftc.teamcode.robots.bobobot;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;

public class MotorDebug {
    DcMotorEx leftBack = null;
    DcMotorEx leftFront = null;
    DcMotorEx rightFront = null;
    DcMotorEx rightBack = null;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public MotorDebug(Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

    }

    public void telemetryOutput(){
        telemetry.addData("rightFront Velocity \t", rightFront.getVelocity());
        telemetry.addData("rightBack velocity \t", rightBack.getVelocity());
        telemetry.addData("leftFront velocity \t", leftFront.getVelocity());
        telemetry.addData("leftBack velocity \t", leftBack.getVelocity());
    }
    long testTime = 0;
    int testStage = 0;
    public void motorDebugTest(double motorPower){
        switch (testStage) {
            case 0:
                testTime = futureTime(2);
                testStage++;

            case 1:
                rightFront.setPower(motorPower);

                if (isPast(testTime)) {
                    rightFront.setPower(0);
                    testTime = futureTime(2);
                    testStage++;
                }
                break;
            case 2:
                rightBack.setPower(motorPower);

                if (isPast(testTime)) {
                    rightBack.setPower(0);
                    testTime = futureTime(2);
                    testStage++;
                }
                break;
            case 3:
                leftBack.setPower(motorPower);

                if (isPast(testTime)) {
                    leftBack.setPower(0);
                    testStage++;
                    testTime = futureTime(2);
                }
                break;
            case 4:
                leftFront.setPower(motorPower);

                if (isPast(testTime)) {
                    leftFront.setPower(0);
                    testStage = 0;
                }
                break;
        }
    }

    public static double motorPower = 0;
    public void runTest(boolean debug, double motorPower) {
        if (debug == true) {
            motorDebugTest(this.motorPower);
        }
    }

}
