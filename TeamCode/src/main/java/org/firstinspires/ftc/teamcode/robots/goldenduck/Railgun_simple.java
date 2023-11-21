package org.firstinspires.ftc.teamcode.robots.goldenduck;

import static org.firstinspires.ftc.teamcode.util.utilMethods.servoNormalize;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
@Config("GoldenDuckGameVariables")
@TeleOp(name="Golden Duck OpMode", group="Challenge")
public abstract class Railgun_simple extends OpMode {
    @Override
    public void loop() {
//        servoRailgun.railgunInit(gamepad1.dpad_right);
        telemetry.update();
    }
        private Servo servoRailgun = null;
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
    public void servoBoolean (boolean press)
    {
        if(press == true)
            servoRailgun.setPosition(servoNormalize(1825));
    }
    public void servoRailgun(Telemetry telemetry, HardwareMap hardwareMap) {
            this.telemetry = telemetry;
            this.hardwareMap = hardwareMap;
        }
        public void railgunInit()
        {
            servoRailgun = this.hardwareMap.get(Servo.class, "servoRailgun");
        }





}
