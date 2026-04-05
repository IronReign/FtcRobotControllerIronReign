
package org.firstinspires.ftc.teamcode.robots.bobby;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;


import java.util.LinkedHashMap;
import java.util.Map;


public class DriveTrain implements Subsystem {
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftFront;


    public DriveTrain(HardwareMap hardwareMap) {
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


        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
    }


    public void mecanumDrive(double forward, double strafe, double turn) {
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;


        double biggest = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        leftBack.setPower(lb / biggest);
        rightBack.setPower(rb / biggest);
        leftFront.setPower(lf / biggest);
        rightFront.setPower(rf / biggest);
    }


    public void slowTurn(boolean right, boolean left) {
        if (right) {
            mecanumDrive(0, 0, 0.2);
        } else if (left) {
            mecanumDrive(0, 0, -0.2);
        }
    }


    public int getLeftFrontTicks() {
        return leftFront.getCurrentPosition();
    }


    public int getRightFrontTicks() {
        return rightFront.getCurrentPosition();
    }


    public int getLeftBackTicks() {
        return leftBack.getCurrentPosition();
    }


    public int getRightBackTicks() {
        return rightBack.getCurrentPosition();
    }


    public double getRightBackPower() {
        return rightBack.getPower();
    }


    @Override
    public void update(Canvas fieldOverlay) {
    }


    @Override
    public void stop() {
        mecanumDrive(0, 0, 0);
    }


    @Override
    public void resetStates() {
        mecanumDrive(0, 0, 0);
    }


    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("LF encoder", getLeftFrontTicks());
        telemetry.put("RF encoder", getRightFrontTicks());
        telemetry.put("LB encoder", getLeftBackTicks());
        telemetry.put("RB encoder", getRightBackTicks());
        return telemetry;
    }


    @Override
    public String getTelemetryName() {
        return "Drive";
    }
}
