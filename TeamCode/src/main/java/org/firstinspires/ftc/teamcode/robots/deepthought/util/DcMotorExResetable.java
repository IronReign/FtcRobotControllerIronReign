package org.firstinspires.ftc.teamcode.robots.deepthought.util;

        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
        import com.qualcomm.robotcore.hardware.PIDCoefficients;
        import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class DcMotorExResetable implements DcMotorEx {
    private DcMotorEx realMotor;
    public int offset = 0;

    public DcMotorExResetable(DcMotorEx realMotor) {
        this.realMotor = realMotor;
    }

    public void setPosition(int position){
        offset = position;
    }

    @Override
    public void setTargetPosition(int position) {
        realMotor.setTargetPosition(position - offset);
    }

    @Override
    public int getTargetPosition() {
        return realMotor.getTargetPosition()+offset;
    }


    @Override
    public int getCurrentPosition() {
        return realMotor.getCurrentPosition()+offset;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        offset = 0;
        realMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void setMotorEnable() {
        realMotor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        realMotor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return realMotor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        realMotor.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        realMotor.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return realMotor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return realMotor.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {

    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        realMotor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        realMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        realMotor.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return null;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return realMotor.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        realMotor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return realMotor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return realMotor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return realMotor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        realMotor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return realMotor.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return realMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        realMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return realMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return realMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        realMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return realMotor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        realMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return realMotor.getPowerFloat();
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void setMode(RunMode mode) {
        realMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return realMotor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        realMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return realMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        realMotor.setPower(power);
    }

    @Override
    public double getPower() {
        return realMotor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return realMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return realMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return realMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return realMotor.getVersion();
    }


    @Override
    public void close() {
        realMotor.close();
    }

}
