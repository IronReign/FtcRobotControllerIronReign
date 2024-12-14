package org.firstinspires.ftc.teamcode.robots.core;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.LinkedHashMap;
import java.util.Map;

/*
Odometry pod has 1440 ticks
Diameter of wheel: 4 in

C = 4pi = 12.566 in
(Distance to travel / 12.566 in)*1440
 */
/*
@Autonomous(name = "COREAUTON2")
public class AutonCode2 extends OpMode {
    Robot robot;
    private FtcDashboard dashboard;
    BNO055IMU imu;
    public static int autonIndex = 1;
    long autonTimer = 0;
    int gpos = 10;
    int spos = 1200;
    float initialzOrientation = 0;
    float nowOrientation = 0;

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

        // Restart motors
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.gearbox.setPower(10);
        robot.gearbox.setVelocity(50);
        robot.gearbox.setTargetPosition(robot.gearboxTargetPosition);
        robot.gearbox.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.slide.setPower(10);
        robot.slide.setVelocity(50);
        robot.slide.setTargetPosition(robot.slideTargetPosition);
        robot.slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.claw.setPosition(robot.clawOpenPosition);

        // Init IMU
        initIMU();
    }

    @Override
    public void loop() {
        switch(autonIndex){
            // Start the robot one tile away facing the bucket with side touching wall
            case 1:
                // Move forward
                robot.mecanumDrive(1,0,0);

                // (5.625/12.566)*1440 (1/4 of a tile)
                if (robot.leftFront.getCurrentPosition() >= 644){
                    autonIndex++;
                }
                break;

            case 2:
                // Gotta Adjust on Wed (repeat in case 8)
                // Angle gearbox
                robot.gearbox.setTargetPosition(gpos);
                // Extend Slide
                robot.slide.setTargetPosition(spos);
                // Open claw
                robot.claw.setPosition(robot.clawOpenPosition);

                if (robot.gearbox.getCurrentPosition()==gpos && robot.slide.getCurrentPosition()==spos && robot.claw.getPosition()==robot.clawOpenPosition) {
                    autonIndex++;
                }

                break;

            case 3:
                // Turn 90 degrees
                initialzOrientation = getZorient();
                robot.mecanumDrive(0, 0, 1);
                nowOrientation = getZorient();
                if (Math.abs(initialzOrientation - nowOrientation) >= 90){
                    autonIndex ++;
                }
                break;

            case 4:
                // Move forward
                robot.mecanumDrive(1,0,0);

                // (45/12.566)*1440 (2 tiles)
                if (robot.leftFront.getCurrentPosition() >= 5156){
                    autonIndex++;
                }
                break;

            case 5:
                // Picks up block
                robot.pickup();

                if (robot.gearbox.getCurrentPosition()==robot.gearboxpick && robot.claw.getPosition()==robot.clawClosePosition) {
                    autonIndex++;
                }
                break;

            case 6:
               // Move backwards
                robot.mecanumDrive(-1,0,0);

                // (45/12.566)*1440 (2 tiles)
                if (robot.leftFront.getCurrentPosition() >= 5156){
                    autonIndex++;
                }
                break;

            case 7:
                // Turn -90 degrees
                initialzOrientation = getZorient();
                robot.mecanumDrive(0, 0, -1);
                nowOrientation = getZorient();

                if (Math.abs(initialzOrientation - nowOrientation) >= 90){
                    initialzOrientation = nowOrientation;
                    autonIndex ++;
                }
                break;

            case 8:
                // Angle gearbox
                robot.gearbox.setTargetPosition(10);
                // Extend Slide
                robot.slide.setTargetPosition(1200);
                // Open claw
                robot.claw.setPosition(robot.clawOpenPosition);
                break;

            default:
                break;
        }

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

}
*/