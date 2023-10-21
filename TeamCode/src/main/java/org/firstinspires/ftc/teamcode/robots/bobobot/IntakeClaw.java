package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeClaw {
    private Servo clawArm = null;
    private Servo clawSpan = null;
    public static double OPENClAW = 0.2;
    public static double CLOSECLAW = 0.05;
    public static double armliftSpeed = 0.1;
    private static int armPosition;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public IntakeClaw(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }


}
