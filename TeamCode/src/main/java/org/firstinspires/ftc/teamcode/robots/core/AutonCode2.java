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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

import java.util.LinkedHashMap;
import java.util.Map;
@Disabled
@Autonomous(name = "COREAUTON2")
public class AutonCode2 extends OpMode {
    Robot robot;
    BNO055IMU imu;
    private FtcDashboard dashboard;
    boolean runAuton = true;
    int autonIndex = 0;
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
    public StickyGamepad spad1;
    boolean reached = false;

    @Override
    public void init() {
        spad1 = new StickyGamepad(gamepad1);
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap, null);


        // Call motors from hardwareMap
        robot.leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        robot.leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        robot.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        robot.rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        robot.shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        robot.claw = hardwareMap.get(Servo.class, "claw");
        robot.slide = hardwareMap.get(DcMotorEx.class, "slide");
        robot.vertical = hardwareMap.get(DcMotorEx.class, "vertical");
        robot.horizontal = hardwareMap.get(DcMotorEx.class, "horizontal");

        // Restart motors
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.vertical.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.horizontal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.shoulder.setPower(1);
        robot.shoulder.setVelocity(50);
        robot.shoulder.setTargetPosition(1500);
        robot.shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.slide.setPower(1);
        robot.slide.setVelocity(50);
        robot.slide.setTargetPosition(0);
        robot.slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.horizontal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.vertical.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.claw.setPosition(robot.clawClosePosition);

        initIMU();
    }

    public void init_loop(){
        debug(new Canvas());

    }

    public void initIMU(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    public float getZorient(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private double kp=0.1, ki=0.0, kd=0.01;
    private double prevError = 0;
    private double integral = 0;

    public double compute(double targeted, double currentValue) {
            double error = targeted - currentValue;
            double pTerm = kp * error;
            integral += error;
            double iTerm = ki * integral;
            double dTerm = kd * (error - prevError);
            double output = pTerm + iTerm + dTerm;
            prevError = error;
            return output;
    }

    public void turn(double degrees, double direction) {
        initialzOrientation = getZorient();
        target = (initialzOrientation + degrees) % 360;
        double control = compute(target, initialzOrientation);
        robot.mecanumDrive(0, 0, direction*control);

        if (Math.abs(nowOrientation-initialzOrientation) <= 2) {
            robot.mecanumDrive(0, 0, 0);
            turning = false;
        }
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
        spad1.update();
        if(spad1.a){
            autonIndex++;
        }
        if(spad1.x){
            autonIndex--;
        }
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
                turn(180, 1);
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
                turn(180, -1);
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
        telemetry.put("Degrees", getZorient());

        telemetry.put("Slide Position", robot.slide.getCurrentPosition());
        telemetry.put("Slide Target Position", robot.slideTargetPosition);
        telemetry.put("Horizontal", robot.horizontal.getCurrentPosition());
        telemetry.put("Vertical", robot.vertical.getCurrentPosition());
        telemetry.put("auton Index", autonIndex);

        return telemetry;
    }
}
