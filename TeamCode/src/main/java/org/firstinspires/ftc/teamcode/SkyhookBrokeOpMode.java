package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp()
public class SkyhookBrokeOpMode extends OpMode {
    public static int skyhookRightTicks, skyhookLeftTicks;
    DcMotorEx skyhookRight, skyhookLeft;
    @Override
    public void init() {
        skyhookLeft = hardwareMap.get(DcMotorEx.class, "kareem");
        skyhookRight = hardwareMap.get(DcMotorEx.class, "jabbar");
        skyhookLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skyhookRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skyhookRight.setTargetPosition(0);
        skyhookLeft.setTargetPosition(0);
        skyhookLeft.setPower(.3);
        skyhookRight.setPower(.3);
        skyhookLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        skyhookRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        skyhookLeft.setTargetPosition(skyhookLeftTicks);
        skyhookRight.setTargetPosition(skyhookRightTicks);
        telemetry.addData("rightexpected", skyhookRightTicks);
        telemetry.addData("leftexpected", skyhookLeftTicks);
        telemetry.addData("right", skyhookRight.getCurrentPosition());
        telemetry.addData("left", skyhookLeft.getCurrentPosition());
    }
}
