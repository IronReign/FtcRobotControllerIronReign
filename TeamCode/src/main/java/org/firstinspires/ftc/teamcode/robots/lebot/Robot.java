package org.firstinspires.ftc.teamcode.robots.lebot;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class Robot {
    public DcMotor leftFront, rightFront;
    public DcMotor intake, conveyor, shooter;
    public Servo paddle, adjustor;

    public Limelight3A limelight;

    private enum shootingState {
        IDLE, SPIN, FEED, FIRE, RESET
    }
    private shootingState shootingState;

    /*
    Gamepad Controls:
    - Button to adjust limelight servo
    - Button for fireBall()
    - PID turning
     */

    private final double intakeSpeed = 1.0;
    private final double conveyorSpeed = 1.0;
    private final double conveyorFeedingSpeed = 0.35;
    private final double shooterPower = 2;

    private final double paddleUp = 0.8;
    private final double paddleDown = 0.2;

    private int numBalls = 0;
    private final int maxBalls = 3;
    int lastBallCount = 0;

    private long timer;

    public void init (HardwareMap hardwareMap){
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        intake = hardwareMap.get(DcMotor.class, "intake");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        paddle = hardwareMap.get(Servo.class, "paddle");
        adjustor = hardwareMap.get(Servo.class, "adjustor");

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        closedChannel();

        intake.setPower(0);
        conveyor.setPower(0);
        shooter.setPower(0);

        shootingState = shootingState.RESET;
    }

    public void intakeOn() {
        if (!channelFull()){
            intake.setPower(intakeSpeed);
            conveyor.setPower(conveyorSpeed);
        } else {
            intakeOff();
        }
    }

    public void intakeOff(){
        intake.setPower(0);
        conveyor.setPower(0);
    }

    public void openChannel(){
        paddle.setPosition(paddleUp);
    }

    public void closedChannel(){
        paddle.setPosition(paddleDown);
    }

    public void feedPaddle(){
        conveyor.setPower(conveyorFeedingSpeed);
        intake.setPower(0);
    }

    // ball detection using limelight
    public void countBalls(){
        int ballsDetected = 0;

        limelight.pipelineSwitch(0); // green
        LLResult greenResult = limelight.getLatestResult();
        if (greenResult.getColorResults() != null){
            ballsDetected += greenResult.getColorResults().size();
        }

        limelight.pipelineSwitch(1); // purple
        LLResult purpleResult = limelight.getLatestResult();
        if (purpleResult.getColorResults() != null){
            ballsDetected += purpleResult.getColorResults().size();
        }

        int newBalls = ballsDetected - lastBallCount;

        if (newBalls > 0){
            numBalls = numBalls + newBalls;
        }

        lastBallCount = ballsDetected;
    }

    public boolean channelFull() {
        return (numBalls == maxBalls);
    }

    public void fireBall(){
        countBalls();

        switch (shootingState){
            case RESET:
                closedChannel();
                intakeOn();
                if (channelFull()){
                    shootingState = shootingState.IDLE;
                }
                break;

            case IDLE:
                intakeOff();
                shooter.setPower(shooterPower);
                timer = futureTime(5);
                shootingState = shootingState.SPIN;
                break;

            case SPIN:
                if(isPast(timer)){
                    feedPaddle();
                    shootingState = shootingState.FEED;
                }
                break;

            case FEED:
                openChannel();
                conveyor.setPower(0);
                timer = futureTime(3);
                shootingState = shootingState.FIRE;
                break;

            case FIRE:
                if (isPast(timer)){
                    numBalls--;
                    closedChannel();
                    if(numBalls == 0){
                        shooter.setPower(0);
                        shootingState = shootingState.RESET;
                    } else {
                        shootingState = shootingState.SPIN;
                    }
                }
                break;
        }
    }
}
