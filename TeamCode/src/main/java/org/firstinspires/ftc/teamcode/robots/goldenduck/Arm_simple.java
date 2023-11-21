package org.firstinspires.ftc.teamcode.robots.goldenduck;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config ("GoldenDuckGameVariables")
@TeleOp(name="Golden Duck OpMode", group="Challenge")

public class Arm_simple {

    class armregistry {
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        private static final float DEADZONE = .1f;

        public armregistry(Telemetry telemetry, HardwareMap hardwareMap) {
            this.telemetry = telemetry;
            this.hardwareMap = hardwareMap;
        }

        public void motorInit() {
            arm = this.hardwareMap.get(DcMotorEx.class, "arm");
            arm.setTargetPosition(0);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm.setPower(0);
        }
    }
        private DcMotor arm = null;
        private int maxArm = Integer.MAX_VALUE;

        public void arm() {
            telemetry.addData("arm position", arm.getCurrentPosition());
        }
        public void armsynapse() {
            if (gamepad1.right_bumper)
                telemetry.addData("arm position", arm.getCurrentPosition() + 0.02 );
            if (gamepad1.left_bumper)
                telemetry.addData("arm position", arm.getCurrentPosition() - 0.02 );
        }

}
