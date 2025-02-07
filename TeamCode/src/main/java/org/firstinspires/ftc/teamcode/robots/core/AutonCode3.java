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

@Autonomous(name = "COREAUTON3")
public class AutonCode3 extends OpMode {
    Robot robot;
    private FtcDashboard dashboard;
    boolean runAuton = true;
    public int autonIndex = 0;
    int startpos = 0;
    public double wheelCircum = ((3.5)*Math.PI);
    public int ticksrev = 1440;
    boolean moving = false;
    boolean turning = false;
    public int targetTicks = 0;
    boolean vertical = true;
    boolean horizontal = false;
    double distance = 0;
    double target = 0;
    double initialzOrientation = 0;
    double nowOrientation = 0;
    public long autonTimer = 0;
    boolean reached = false;

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

    public void forward(double length, double direction){
        if (!moving){
            // Number of encoder ticks per distance
            targetTicks = (int)((length/wheelCircum)*ticksrev);

            // Assign initial encoder values
            startpos = robot.vertical.getCurrentPosition();

            // Indicate Vertical/Horizontal
            vertical = true;
            horizontal = false;

            // Travel Distance
            robot.mecanumDrive(-direction,0,0);

            // Update moving
            moving = true;
        }
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

    public void debug(Canvas fieldOverlay){
        autonIndex = robot.debugAuton(autonIndex);
        handleTelemetry(getTelemetry(true), robot.getTelemetryName());
    }

    public boolean completed(){
        if (moving) {
            if (vertical){
                distance = robot.vertical.getCurrentPosition()-startpos;
            }

            else if (horizontal){
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
                //literally everything setup
                //TODO: shoulder adjust; hunch its too low
                //TODO: timing
                //5:30-7:30
                robot.claw.setPosition(robot.clawClosePosition);
                forward(69, 0.04); //OG: 60
                robot.shoulder.setTargetPosition(1785);//OG: 275, 220+1647=1867
                robot.slide.setTargetPosition(350); //444
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0, 0, 0);
                    autonTimer = futureTime(1);
                }
                break;

            case 1:
                //secure pres
                if (isPast(autonTimer)) {
                    robot.shoulder.setTargetPosition(1360);
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

            case 3:
                //Back up
                forward(50, -0.04); //OG: 60
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0, 0, 0);
                }

                break;

            // Specimen dos
            //secure the president

            case 4:
                //turn 180
                //TODO: pid works?
                robot.turnUntilDegreesIMU(180, 1);
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0, 0, 0);
                }

                break;

            case 5:
                strafe(35, -1);
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0,0,0);
                }

            case 6:
                forward(19, 0.04);
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0,0,0);
                }

            case 7:
                //close
                robot.claw.setPosition(robot.clawClosePosition);
                //TODO: edit here based on 1
                robot.shoulder.setTargetPosition(1785);
                forward(19, -0.04);
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0,0,0);
                }
                break;

            //back to submersible
            case 8:
                robot.turnUntilDegreesIMU(180, 1);
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0, 0, 0);
                }

            case 9:
                strafe(35, -1);
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0,0,0);
                }

                //attach on high
                //TODO: edit based on 1
            case 10:
                forward(50, 0.04); //OG: 60
                robot.shoulder.setTargetPosition(1785);//OG: 275, 220+1647=1867
                robot.slide.setTargetPosition(350); //444
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0, 0, 0);
                    autonTimer = futureTime(1);
                }
                break;

            case 11:
                //secure specimen
                if (isPast(autonTimer)) {
                    robot.shoulder.setTargetPosition(1360);
                    if (robot.shoulder.getCurrentPosition() <= robot.shoulder.getTargetPosition()) {
                        autonIndex++;
                        autonTimer = futureTime(1);
                    }
                }
                break;

            case 12:
                //open claw
                if (isPast(autonTimer)) {
                    robot.claw.setPosition(robot.clawOpenPosition);
                    autonIndex++;
                }
                break;

            case 13:
                //Back up
                forward(50, -0.04); //OG: 60
                robot.slide.setTargetPosition(0);
                if (completed()) {
                    autonIndex++;
                    robot.mecanumDrive(0, 0, 0);
                }

                // Park
            case 14:
                strafe(35, -1);
                robot.slide.setTargetPosition(0);
                if(completed()){
                    autonIndex++;
                    robot.mecanumDrive(0,0,0);
                    autonIndex = 0;
                    return true;
                }

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
        telemetry.put("Target Ticks", targetTicks);
        telemetry.put("Moving", moving);
        telemetry.put("Distance", distance);
        telemetry.put("Reached", reached);
        telemetry.put("Degrees", robot.getZorient()); // you want to comment this out - getting the imu heading is expensive - get only when needed

        telemetry.put("Slide Position", robot.slide.getCurrentPosition());
        telemetry.put("Slide Target Position", robot.slideTargetPosition);
        telemetry.put("Horizontal", robot.horizontal.getCurrentPosition());
        telemetry.put("Vertical", robot.vertical.getCurrentPosition());
        telemetry.put("auton Index", autonIndex);

        return telemetry;
    }
}