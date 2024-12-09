package org.firstinspires.ftc.teamcode.robots.core;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.Map;

@Autonomous(name = "COREAUTON")
public class CoreAuton extends OpMode {
    Robot robot;
    private FtcDashboard dashboard;

    public static int autonIndex = 0;
    long autonTimer = futureTime(10);
    public static double adjust_time = 1.0;
    public static double strafeOne = 0.6;
    public static double time_ninety = 0.55;
    public static int timeTurn = 1;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap, null);
        robot.init();
    }

    @Override
    public void loop() {
        execute();

        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        TelemetryPacket p = new TelemetryPacket();
        telemetry.addLine(telemetryName);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
            p.addLine(line);
        }
        telemetry.addLine();
        dashboard.sendTelemetryPacket(p);
    }

    public void execute() {

        switch (autonIndex) {
            case 0:
                autonTimer = futureTime(strafeOne);
                autonIndex++;
                break;
            case 1:
                // Strafe to the left side
                robot.mecanumDrive(0,-1,0);
                //if (rightBack.getCurrentPosition() == rightBack.getTargetPosition()){
                if (isPast(autonTimer)) {
                    autonIndex++;
                    autonTimer = futureTime(time_ninety);
                }
                //}
                // NUMBER OF TICKS (AMOUNT) IT MOVES PER FOOT

                break;
            case 2:
                // Turn -90 degrees
                robot.mecanumDrive(0,0,-1);
                //rightBack.setTargetPosition(100);

                if (isPast(autonTimer)){
                autonIndex += 3;
                autonTimer = futureTime(0.4);
                }

                break;
            case 3:
                /*
                // Extend Linear Slide
                if (isPast(autonTimer)){
                    slide.setTargetPosition(slideUp);
                    //shoulder.setTargetPosition(shoulderUp);
                    elbow.setTargetPosition(elbowUp);
                    if ((slide.getCurrentPosition() == slide.getTargetPosition())&(elbow.getCurrentPosition()==elbow.getTargetPosition())){
                        autonIndex++;
                    }
                }
                break;*/
            case 4:
                /*// Open claw to release block
                claw.setPosition(clawOpenPosition);
                autonIndex++;
                break;*/
            case 5:
                // Strafe Right
                robot.mecanumDrive(0,1,0);
                //rightBack.setTargetPosition(100);

                if (isPast(autonTimer)) {
                    autonIndex++;
                    autonTimer = futureTime(time_ninety);
                    //}
                    break;
                }

            case 6:
                // Turn 90
                robot.mecanumDrive(0,1,0);
                //rightBack.setTargetPosition(100);

                //if (rightBack.getCurrentPosition() == rightBack.getTargetPosition()){
                autonIndex++;
                //}
                break;

            /*case 6:
                // Turn 180
                robot.mecanumDrive(0,0,1);
                //rightBack.setTargetPosition(100);

                //if (rightBack.getCurrentPosition() == rightBack.getTargetPosition()){
                autonIndex++;
                //}

                break;

            case 7:

                // Return to normal
                slide.setTargetPosition(slideTargetPosition);
                //shoulder.setTargetPosition(shoulderTargetPosition);
                elbow.setTargetPosition(elbowTargetPosition);

                robot.leftFront.setPower(0); //RIGHTFRONT
                robot.rightFront.setPower(0); //LEFT FRONT
                robot.leftBack.setPower(0); //RIGHTBACK
                robot.rightBack.setPower(0);

                break;
*/
            /*case 8:
                // Move a unit
                if (isPast(autonTimer)){
                    leftFront.setPower(powerOne);
                    rightFront.setPower(-powerOne);
                    leftBack.setPower(powerOne);
                    rightBack.setPower(-powerOne);
                    autonTimer = futureTime(timeOne);
                    autonIndex++;
                }
                break;

            case 9:
                // Extend Slide angled down
                if (isPast(autonTimer)) {
                    slide.setTargetPosition(slideUp);
                    shoulder.setTargetPosition(shoulderDown);
                    elbow.setTargetPosition(elbowDown);
                    if ((slide.getCurrentPosition() == slide.getTargetPosition()) & (shoulder.getCurrentPosition() == shoulder.getTargetPosition()) & (elbow.getCurrentPosition() == elbow.getTargetPosition())) {
                        autonIndex++;
                    }
                }
                break;

            case 10:
                // Close claw (hold brick)
                claw.setPosition(clawClosePosition);
                autonIndex++;
                break;

            case 11:
                // Retract slide
                slide.setTargetPosition(slideRe);
                if ((slide.getCurrentPosition() == slide.getTargetPosition())){
                    autonIndex++;
                }
                break;

            case 12:
                // Move back a unit
                leftFront.setPower(-powerOne);
                rightFront.setPower(powerOne);
                leftBack.setPower(-powerOne);
                rightBack.setPower(powerOne);
                autonTimer = futureTime(timeOne);
                autonIndex++;
                break;

            case 13:
                // Turn -180 degrees
                if (isPast(autonTimer)) {
                    leftFront.setPower(powerOne);
                    rightFront.setPower(powerOne);
                    leftBack.setPower(powerOne);
                    rightBack.setPower(powerOne);
                    autonTimer = futureTime((float) (timeOne * 4) /3);
                    autonIndex++;
                    //autonIndex=autonIndex-13;
                    //adjust_time = 0.75;
                }
                break;*/

            default:
                break;
        }
    }
}