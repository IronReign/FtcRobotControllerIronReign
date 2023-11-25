//package org.firstinspires.ftc.teamcode.robots.goldenduck;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//@Config ("GoldenDuckGameVariables")
//
//public class Arm {
//
//    class armregistry {
//        private Telemetry telemetry;
//        private HardwareMap hardwareMap;
//        private DcMotor arm = null;
//        public armregistry(Telemetry telemetry, HardwareMap hardwareMap) {
//            this.telemetry = telemetry;
//            this.hardwareMap = hardwareMap;
//        }
//        public void motorInit() {
//            arm = this.hardwareMap.get(DcMotorEx.class, "arm");
//            arm.setTargetPosition(0);
//            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            arm.setPower(0);
//        }
//    }
//}
