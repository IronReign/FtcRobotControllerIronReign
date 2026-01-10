package org.firstinspires.ftc.teamcode.robots.core;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDController;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.LinkedHashMap;
import java.util.Map;

@Disabled

@Autonomous(name = "COREAUTON4")
public class AutonCode4 extends OpMode {
    Robot robot;
    private FtcDashboard dashboard;
    boolean runAuton = true;
    public int autonIndex = 0;
    public long autonTimer = 0;
    int startpos = 0;
    public double wheelCircum = ((3.5)*Math.PI);
    public int ticksrev = 1440;
    boolean moving = false;
    boolean turning = false;
    public int targetTicks = 0;
    boolean vertical = true;
    boolean horizontal = false;
    double distance = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        robot.init();
        dashboard = FtcDashboard.getInstance();
    }

    public void init_loop(){
        debug(new Canvas());
        robot.initloopDrive();

    }

    public void debug(Canvas fieldOverlay){
        autonIndex = robot.debugAuton(autonIndex);
        handleTelemetry(getTelemetry(true), robot.getTelemetryName());
    }

    public void strafe(double length, double direction){
        if (!moving){
            // Number of encoder ticks per distance
            targetTicks = (int)((length/wheelCircum)*ticksrev);

            // Assign initial encoder values
            startpos = robot.horizontal.getCurrentPosition();

            // Indicate Vertical/Horizontal
            vertical = false;
            horizontal = true;

            // Travel Distance
            robot.mecanumDrive(0, direction,0);

            // Update moving
            moving = true;
        }
    }

    public boolean completed(){
        if (moving) {
            if (horizontal){
                distance = robot.horizontal.getCurrentPosition()-startpos;
            }

            if (Math.abs(distance) >= Math.abs(targetTicks)) {
                robot.mecanumDrive(0, 0, 0);
                vertical = false;
                horizontal = false;
                moving = false;
                return true;

            }
        }
        return false;
    }

    public boolean execute(){
        switch(autonIndex) {
            // Starting Position: A3 facing submersible with specimen in hand
            // Specimen one
            case 0:
                //TODO: shoulder adjust; hunch its too low
                robot.claw.setPosition(robot.clawClosePosition);
                if (robot.driveDistance(20, 0.25)){
                    robot.shoulder.setTargetPosition(1800);
                    robot.slide.setTargetPosition(315);
                    autonIndex++;
                    autonTimer = futureTime(1);
                }
                break;

            case 1:
                if (isPast(autonTimer)) {
                    robot.shoulder.setTargetPosition(1400);
                    if (robot.shoulder.getCurrentPosition() <= robot.shoulder.getTargetPosition()) {
                        autonIndex++;
                        autonTimer = futureTime(1);
                    }
                }
                break;

            case 2:
                //open claw
                if (isPast(autonTimer)) {
                    robot.claw.setPosition(robot.clawOpenPosition);
                    autonIndex++;
                }
                break;

            /*case 3:
                //Back up
                robot.driveDistance((85-42), -1);
                robot.shoulder.setTargetPosition(950);
                robot.slide.setTargetPosition(110);
                autonIndex++;
                break;

            // Specimen dos

            case 4:
                //turn 180
                robot.turnUntilDegreesIMU(180, 1);
                autonIndex++;
                break;

            case 5:
                strafe(35, -1);
                robot.driveDistance(5, 1);
                autonIndex++;
                break;

            case 6:
                //close
                robot.claw.setPosition(robot.clawClosePosition);
                //TODO: edit here based on 1
                robot.shoulder.setTargetPosition(1785);
                //turns 180
                robot.turnUntilDegreesIMU(180, 1);
                autonIndex++;
                break;

            //back to submersible
            case 7:
                robot.strafeDistance(24, -1);
                robot.driveDistance(30, 1);
                autonIndex++;
                break;

            case 8:
                //TODO: edit here based on 1
                robot.shoulder.setTargetPosition(1785);
                robot.slide.setTargetPosition(350);
                autonIndex++;
                autonTimer = futureTime(1);
                break;

            //attach on high
            //TODO: edit based on 1
            case 9:
                if (isPast(autonTimer)) {
                    robot.shoulder.setTargetPosition(1360);
                    if (robot.shoulder.getCurrentPosition() <= robot.shoulder.getTargetPosition()) {
                        autonIndex++;
                        autonTimer = futureTime(1);
                    }
                }
                break;

            case 10:
                //open claw
                if (isPast(autonTimer)) {
                    robot.claw.setPosition(robot.clawOpenPosition);
                    autonIndex++;
                }
                break;

            // pushing in starts

            case 11:
                //Back up
                robot.driveDistance(6, -1);
                robot.slide.setTargetPosition(0);
                autonIndex++;


            case 12:
                //move to the right towards blocks
                robot.strafeDistance(36, -1);
                robot.slide.setTargetPosition(0);
                autonIndex++;

            case 13:
                //go past
                robot.driveDistance(30,1);
                autonIndex++;
                break;

            case 14:
                //turn 180
                robot.strafeDistance(12,1);
                robot.driveDistance(45,-1);
                autonIndex =0;
                return true;*/

            default:
                break;
        }
        return false;
    }

    @Override
    public void loop() {
        debug(new Canvas());
        if (runAuton) {
            runAuton = !execute();
        }
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

    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Claw Open", robot.clawOpen);
        telemetry.put("Shoulder Power", robot.shoulder.getCurrent(CurrentUnit.AMPS));
        telemetry.put("Shoulder Position", robot.shoulder.getCurrentPosition());
        telemetry.put("Shoulder Target Position", robot.shoulder.getTargetPosition());
        telemetry.put("Shoulder runMode", robot.shoulder.getMode());

        telemetry.put("Power", robot.leftFront.getPower());
        //telemetry.put("Degrees", robot.getZorient()); // you want to comment this out - getting the imu heading is expensive - get only when needed

        telemetry.put("Slide Position", robot.slide.getCurrentPosition());
        telemetry.put("Slide Target Position", robot.slideTargetPosition);
        telemetry.put("Horizontal", robot.horizontal.getCurrentPosition());
        telemetry.put("Vertical", robot.vertical.getCurrentPosition());
        telemetry.put("auton Index", autonIndex);

        return telemetry;
    }
}
