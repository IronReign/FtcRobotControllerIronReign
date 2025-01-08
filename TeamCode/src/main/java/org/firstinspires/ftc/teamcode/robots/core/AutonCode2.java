package org.firstinspires.ftc.teamcode.robots.core;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.LinkedHashMap;
import java.util.Map;

@Autonomous(name = "COREAUTON2")
public class AutonCode2 extends OpMode {
    Robot robot;
    BNO055IMU imu;
    private FtcDashboard dashboard;
    public static int autonIndex = 0;
    int startpos = 0;
    public double wheelCircum = ((3.5)*Math.PI);
    public int ticksrev = 1440;
    boolean moving = false;
    boolean turning = false;
    public int ticks = 0;
    boolean vertical = true;
    boolean horizontal = false;
    double distance = 0;
    double target = 0;
    double initialzOrientation = 0;
    double nowOrientation = 0;

    @Override
    public void init() {
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
        robot.shoulder.setTargetPosition(robot.shoulderTargetPosition);
        robot.shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.slide.setPower(1);
        robot.slide.setVelocity(50);
        robot.slide.setTargetPosition(robot.slideTargetPosition);
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

    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Orientation Angle", getZorient());

        return telemetry;
    }

    public void forward(double length, double direction){
        if (!moving){
            // Number of encoder ticks per distance
            ticks = (int)((length/wheelCircum)*ticksrev);

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
            ticks = (int)((length/wheelCircum)*ticksrev);

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

    public void turn(double degrees, int direction) {
        initialzOrientation = getZorient();
        target = (initialzOrientation + degrees) % 360;
        robot.mecanumDrive(0, 0, direction);
        turning = true;
    }

    public void check(){
        if (moving) {
            if (vertical){
                distance = robot.vertical.getCurrentPosition()-startpos;
            }

            else if (horizontal){
                distance = robot.horizontal.getCurrentPosition()-startpos;
            }

            if (Math.abs(distance) >= Math.abs(ticks)){
                robot.mecanumDrive(0,0,0);
                vertical = false;
                horizontal = false;
                moving = false;
                autonIndex++;
            }
        }

        if(turning){
            nowOrientation = (getZorient()) % 360;

            if(Math.abs(nowOrientation-initialzOrientation) >= target){
                robot.mecanumDrive(0,0,0);
                turning = false;
                autonIndex++;
            }
        }
    }

    @Override
    public void loop() {
        check();
        switch(autonIndex){
            // Starting Position: A3 facing submersible with specimen in hand
            case 0:
                // Adjust shoulder and slide position
                // TODO: Figure out slide position

                robot.slide.setTargetPosition(500);
                robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() + 275);
                autonIndex++;
                break;

            case 1:
                // Move forward 0.8 tile
                forward((65), 0.04);
                break;

            case 2:
                // Push shoulder down
                robot.shoulder.setPower(70);
                robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() - 300);
                robot.slide.setTargetPosition(robot.slide.getCurrentPosition()+40);

                autonIndex+=2;
                break;



           case 4:
                //Move backwards
                forward(20, -1);

            case 5:
                // Open claw
                robot.claw.setPosition(robot.clawOpenPosition);
                autonIndex++;
                break;

            case 6:
                // Move backwards one tile
                forward(4, -0.1);
                break;



            default:
                break;
        }
    }
}
