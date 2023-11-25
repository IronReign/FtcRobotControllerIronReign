package org.firstinspires.ftc.teamcode.robots.goldenduck;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist {
    Servo clawWrist = null;
    private Telemetry telemetry;
    public void clawWrist() {
        telemetry.addData("Claw wrist position:", clawWrist.getPosition());

    }
}
