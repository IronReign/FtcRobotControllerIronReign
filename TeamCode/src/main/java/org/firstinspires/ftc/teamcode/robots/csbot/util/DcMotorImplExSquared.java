package org.firstinspires.ftc.teamcode.robots.csbot.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import androidx.annotation.NonNull;

@Config
public class DcMotorImplExSquared extends DcMotorImplEx implements DcMotorExSquared {

    private int offset;

    public DcMotorImplExSquared(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
    }

    public void resetPosition(int position){
        offset = super.getCurrentPosition()+position;
    }
    @Override
    public void setTargetPosition(int position) {
        super.setTargetPosition(position - offset);
    }

    @Override
    public int getTargetPosition() {
        return super.getTargetPosition() + offset;
    }


    @Override
    public int getCurrentPosition() {

        return super.getCurrentPosition() + offset;
    }


    @Override
    public void resetDeviceConfigurationForOpMode() {
        offset = 0;
        super.resetDeviceConfigurationForOpMode();
    }

}
