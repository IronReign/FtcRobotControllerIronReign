package org.firstinspires.ftc.teamcode.robots.bobby2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Timer;


@Config(value= "enrique")
public class robot2 implements Subsystem {
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    DcMotorEx leftBack, rightBack, rightFront, leftFront;


    public robot2(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        slowTurn(gamepad1.dpad_right,gamepad1.dpad_left);
    }
    public void mecanumDrive(double forward, double strafe, double turn) {


        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        //turn stuff to be less than 1.0
        double biggest = Math.max(  1.0 ,   Math.max(    Math.abs(lf)   ,  Math.max(  Math.abs(rf) , Math.max( Math.abs(lb) , Math.abs(rb)  )  )   )   );

        leftBack.setPower( lb / biggest);
        rightBack.setPower( rb / biggest);
        leftFront.setPower( lf / biggest);
        rightFront.setPower( rf/ biggest);
    }

    public void slowTurn(boolean right, boolean left) {
        if(right) mecanumDrive(0,0,0.2);
        else if(left) mecanumDrive(0,0,-0.2);
    }

    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");


        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        //1 - R
        //2 - L
    }
    @Override
    public void stop() {
    }
    @Override
    public void resetStates() {
    }
    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();
        return telemetry;
    }
    @Override
    public String getTelemetryName(){
        return "bobby robot";
    }
    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}