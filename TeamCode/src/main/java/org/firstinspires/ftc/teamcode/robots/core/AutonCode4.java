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

@Autonomous(name = "COREAUTON4")
public class AutonCode4 extends OpMode {
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

    public void debug(Canvas fieldOverlay){
        autonIndex = robot.debugAuton(autonIndex);
        handleTelemetry(getTelemetry(true), robot.getTelemetryName());
    }

    public boolean execute(){
        switch(autonIndex) {
            // Starting Position: A3 facing submersible with specimen in hand
            // Specimen one
            case 0:
                //TODO: shoulder adjust; hunch its too low
                robot.claw.setPosition(robot.clawClosePosition);
                robot.driveDistance(30, 1);
                robot.shoulder.setTargetPosition(1785);
                robot.slide.setTargetPosition(350);
                autonIndex++;
                autonTimer = futureTime(1);
                break;

            case 1:
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
                robot.driveDistance(6, -1);
                robot.shoulder.setTargetPosition(824);
                robot.slide.setTargetPosition(30);
                autonIndex++;
                break;

            // Specimen dos

            case 4:
                //turn 180
                robot.turnUntilDegreesIMU(180, 1);
                autonIndex++;
                break;

            case 5:
                robot.strafeDistance(24, -1);
                robot.driveDistance(24, 1);
                autonIndex+=2;
                break;

           /* case 6:
                robot.driveDistance(24, 1);
                autonIndex++;
                break;*/

            case 7:
                //close
                robot.claw.setPosition(robot.clawClosePosition);
                //TODO: edit here based on 1
                robot.shoulder.setTargetPosition(1785);
                //turns 180
                robot.turnUntilDegreesIMU(180, 1);
                autonIndex++;
                break;

            //back to submersible
            case 8:
                robot.strafeDistance(24, -1);
                robot.driveDistance(30, 1);
                autonIndex++;
                break;

            case 9:
                //TODO: edit here based on 1
                robot.shoulder.setTargetPosition(1785);
                robot.slide.setTargetPosition(350);
                autonIndex++;
                autonTimer = futureTime(1);
                break;

            //attach on high
            //TODO: edit based on 1
            case 10:
                if (isPast(autonTimer)) {
                    robot.shoulder.setTargetPosition(1360);
                    if (robot.shoulder.getCurrentPosition() <= robot.shoulder.getTargetPosition()) {
                        autonIndex++;
                        autonTimer = futureTime(1);
                    }
                }
                break;

            case 11:
                //open claw
                if (isPast(autonTimer)) {
                    robot.claw.setPosition(robot.clawOpenPosition);
                    autonIndex++;
                }
                break;

            case 12:
                //Back up
                robot.driveDistance(30, -1);
                robot.slide.setTargetPosition(0);
                autonIndex++;

            // Park
            case 13:
                robot.strafeDistance(24, -1);
                robot.slide.setTargetPosition(0);
                autonIndex = 0;
                return true;

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
