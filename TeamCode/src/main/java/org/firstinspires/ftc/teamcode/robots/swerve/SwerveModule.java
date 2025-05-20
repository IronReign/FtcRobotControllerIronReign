package org.firstinspires.ftc.teamcode.robots.swerve;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;
import org.firstinspires.ftc.teamcode.util.utilMethods;

public class SwerveModule {
    private DcMotorEx driveMotor;
    private CRServo yawServo;
    private DcMotorEx yawEncoder;
    private PIDController yawPID;
    private double ticksPerDegree;
    public boolean swerveAligned;

    private double targetAngle;       // in degrees, as a wrapped value (0-360)
    private double yawError;          // current yaw error (in degrees)
    private double thresholdAngle;    // if error is greater than this, do not enable drive
    private double desiredAnglePrivy;
    private boolean invertedDrive;    // whether the drive motor should be inverted

    /**
     * @param driveMotor      The drive motor (for wheel propulsion)
     * @param yawServo        The continuous rotation servo that steers the module
     * @param yawEncoder      The encoder providing feedback on the module’s yaw position
     * @param pidController   The PID controller instance to control the yaw
     * @param ticksPerDegree  Conversion factor from encoder ticks to degrees
     * @param thresholdAngle  Threshold (in degrees) below which the drive motor is enabled
     */
    public SwerveModule(DcMotorEx driveMotor, CRServo yawServo, DcMotorEx yawEncoder,
                        PIDController pidController, double ticksPerDegree, double thresholdAngle) {
        this.driveMotor = driveMotor;
        this.yawServo = yawServo;
        this.yawEncoder = yawEncoder;
        this.yawPID = pidController;
        this.ticksPerDegree = ticksPerDegree;
        this.thresholdAngle = thresholdAngle;
        this.invertedDrive = false;
    }

    /**
     * Call this periodically to update the yaw servo power.
     */
    public void update() {
        double currentAngle = getCurrentAngle();
        // Compute the current error using a helper that returns the smallest difference (-180, 180)
        yawError = utilMethods.angleDifference(targetAngle, currentAngle);
        yawPID.setInput(currentAngle);
        yawPID.setSetpoint(targetAngle);
        double pidOutput = yawPID.performPID();
        // Negative because of the servo’s orientation (adjust sign as needed)
        yawServo.setPower(-pidOutput);
    }

    /**
     * Sets the desired state for the module.
     * @param desiredAngle Desired chassis–relative angle (in degrees)
     * @param speed        Desired speed (value from 0 to 1)
     */
    public void setDesiredState(double desiredAngle, double speed) {
        // Used for desiredAngle telemetry
        desiredAnglePrivy = desiredAngle;
        double currentAngle = getCurrentAngle();
        // Compute the minimal angle difference (–180 to 180)
        double angleDiff = utilMethods.angleDifference(desiredAngle, currentAngle);
//easter egg
         //If rotating more than 90° is required, invert drive to minimize rotation.
        if (Math.abs(angleDiff) >= 90) {
            invertedDrive = true;
            targetAngle = Utils.wrapAngle(desiredAngle - 180);
        } else {
            invertedDrive = false;
            targetAngle = Utils.wrapAngle(desiredAngle);
        }


        // Optionally, only allow the drive motor to run if the yaw error is within threshold or about opposite of the current angle.
        if (Math.abs(angleDiff) < thresholdAngle || Math.abs(angleDiff) > 180 - thresholdAngle && Math.abs(angleDiff) < 180 + thresholdAngle) {
            swerveAligned = true;
        } else {
            swerveAligned = false;
            driveMotor.setPower(0);
        }
                driveMotor.setPower(invertedDrive? -speed : speed);



    }

    public void resetEncoder () {
        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Optionally, only allow the drive motor to run if the yaw error is within threshold.
//        if (Math.abs(angleDiff) < thresholdAngle) {
           // driveMotor.setPower(invertedDrive ? -speed : speed);
//        } else {
//            driveMotor.setPower(0);
//        }
    }

    /**
     * Returns the current yaw angle of the module, in degrees (wrapped between 0 and 360).
     */

    public void aligned(double speed){
        driveMotor.setPower(invertedDrive ? -speed : speed);
    }

    public double getCurrentAngle() {
        return Utils.wrapAngle(yawEncoder.getCurrentPosition() / ticksPerDegree);
    }

    /**
     * Returns the target angle that the module is trying to achieve.
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Returns the current yaw error (in degrees).
     */
    public double getYawError() {
        return yawError;
    }

    /**
     * Returns the current drive power setting.
     */
    public double getDrivePowerActual() {
        return driveMotor.getPower();
    }
    /**
     * Returns the current drive power setting.
     */
    public double getDriveAmps() {
        return driveMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getDesiredAngle() {
        return desiredAnglePrivy;
    }

    public void resetEncoders() {
        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
