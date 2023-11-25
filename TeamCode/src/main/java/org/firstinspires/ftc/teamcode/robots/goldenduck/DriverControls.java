package org.firstinspires.ftc.teamcode.robots.goldenduck;

import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Config ("GoldenDuckGameVariables")
@TeleOp(name="Golden Duck OpMode", group="Challenge")
public class DriverControls extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public boolean auton = true;
    public static boolean testing = false;
    public static boolean red = true;
    public static boolean calibrateOn = true;
    private boolean calibrate = false;
    public static float DEADZONE = .1f;
    static final double FEET_PER_METER = 3.28084;
    int tagDetected = 0;
    DriveTrain driveTrain;
    RobotReference ggd;
    Servo servoClaw;
    Servo clawWrist;
    private DcMotor arm = null;



    @Override
    public void init() {
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        ggd = new RobotReference(telemetry, hardwareMap);
      driveTrain = new DriveTrain(telemetry, hardwareMap);
      driveTrain.motorInit();
      clawWrist = hardwareMap.get(Servo.class, "servoWrist");
      arm = this.hardwareMap.get
              (DcMotorEx.class, "arm");
      arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
      telemetry.addData("arm position", arm.getCurrentPosition());
    }
    @Override
    public void loop() {
        telemetry.addData("servoWrist", clawWrist.getPosition());
        telemetry.addData("servoClaw", servoClaw.getPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());

        driveTrain.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.dpad_down) {
            calibrate = false;
        }
        if (gamepad1.dpad_up) {
            if (driveTrain.robotSpeed == 1)
                driveTrain.robotSpeed = .5;
            else
                driveTrain.robotSpeed = 1;
        }
        ggd.droneShot.droneGo(gamepad1.dpad_left);
        ggd.droneShot.telemetryOutput();
        //open claw
        if (gamepad1.x) {
            arm.setPower(0.1);
            arm.setTargetPosition(-105);
            clawWrist.setPosition(servoNormalize(1047));
        }
        if (gamepad1.y) {
            arm.setPower(0.2);
            arm.setTargetPosition(-100);
            clawWrist.setPosition(servoNormalize(1277));
//      y is the like the mid section or to like pick up pixels
        }
        if (gamepad1.b) {
            arm.setPower(0.3);
            arm.setTargetPosition(-1050);
            clawWrist.setPosition(0);
        }
        if (gamepad1.a) {
            arm.setPower(0.3);
            arm.setTargetPosition(-1023);
            clawWrist.setPosition(0);
        }
//      b is the high position to score backwards
        if (gamepad1.right_bumper) {
            servoClaw.setPosition(servoNormalize(1711));
        }
        // claw open
        if (gamepad1.left_bumper){
            servoClaw.setPosition(servoNormalize(1821));
        }
        //claw close
    }
}