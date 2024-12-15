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
    float initialzOrientation = 0;
    float nowOrientation = 0;

    /*
    Notes to Self:
    - IMU is used for rotation
    - Encoder ticks are used for forward and strafing movements
     */


    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap, null);

        // Call motors from hardwareMap
        robot.leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        robot.leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        robot.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        robot.rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        robot.shoulder = hardwareMap.get(DcMotorEx.class, "gearbox");
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

        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.horizontal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.vertical.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.claw.setPosition(robot.clawOpenPosition);

        // Init IMU
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

    // Wheels
    public double wheelCircum = ((4.09449)*Math.PI);
    public int ticksrev = 1440;

    public void forward(double length, int direction){
        // Number of encoder ticks per distance
        int ticks = (int)((length/wheelCircum)*ticksrev);

        // Reset pod encoders
        robot.vertical.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Move chassis motors
        robot.mecanumDrive(direction,0,0);

        // Stop after Distance is moved
        if (Math.abs(robot.vertical.getCurrentPosition()) >= Math.abs(ticks)){
            robot.mecanumDrive(0,0,0);
        }

    }

    public void strafe(double length, int direction){
        // Number of encoder ticks per distance
        int ticks = (int)((length/wheelCircum)*ticksrev);

        // Reset pod encoders
        robot.horizontal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Move chassis motors
        robot.mecanumDrive(0, direction,0);

        // Stop after Distance is moved
        if (Math.abs(robot.horizontal.getCurrentPosition()) >= Math.abs(ticks)){
            robot.mecanumDrive(0,0,0);
        }
    }


    @Override
    public void loop() {
        switch(autonIndex){
            // Starting Position: A3 facing opposite from the bucket
            case 0:
                // Adjust shoulder, slide, and claw position
                // TODO: Figure out shoulder position
                robot.shoulder.setTargetPosition(700);
                robot.claw.setPosition(robot.clawOpenPosition);
                // TODO: Figure out slide position
                robot.slide.setTargetPosition(0);

            case 1:
                // Move Forward One Tile
                forward(24, 1);
                autonIndex++;
                break;

            case 2:
                // Close Claw
                robot.claw.setPosition(robot.clawClosePosition);

            case 3:
                // Turn 90 Degrees Counter Clockwise
                // TODO: Define turning function
                initialzOrientation = getZorient();
                robot.mecanumDrive(0, 0, -1);
                nowOrientation = getZorient();

                if (Math.abs(initialzOrientation - nowOrientation) >= 90){
                    initialzOrientation = nowOrientation;
                    autonIndex ++;
                }
                break;

            case 4:
                // Strafe Left
                strafe(24, -1);
                autonIndex++;
                break;

            case 5:
                // Move Forward 0.75 Tile
                forward((24*0.75), 1);
                autonIndex++;
                break;

            case 6:
                // Push Shoulder Down
                robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() - 75);
                autonIndex++;
                break;

            case 7:
                // Move Forward 0.25 Tile
                forward((24*0.25), 1);
                autonIndex++;
                break;

            case 8:
                // Adjust shoulder, slide, and claw position to init position
                robot.shoulder.setTargetPosition(700);
                robot.claw.setPosition(robot.clawOpenPosition);
                // TODO: Figure out slide init position
                robot.slide.setTargetPosition(0);

            default:
                break;
        }

    }
}
