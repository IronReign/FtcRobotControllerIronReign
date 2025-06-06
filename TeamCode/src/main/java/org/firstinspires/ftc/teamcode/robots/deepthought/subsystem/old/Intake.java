package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.old;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "AA_ITD_INTAKE")
public class Intake implements Subsystem {
    public static int RIGHT_DIVERTER_OPEN = 1850;
    public static int LEFT_DIVERTER_OPEN = 1050;
    public static int LEFT_DIVERTER_CLOSED = 1650;
    public static int RIGHT_DIVERTER_CLOSED = 1050;
    public static int ANGLE_GROUND = 2020; //where the intake hits the ground
    public static int ANGLE_INGEST_INCREMENT = 20;
    public static int ANGLE_START = 995;
    public static int ANGLE_PWM_MAX = ANGLE_GROUND + ANGLE_INGEST_INCREMENT; //just below ground
    public static int ANGLE_PWM_MIN = ANGLE_START;
    public static int ANGLE_INGEST_GROUND = ANGLE_GROUND;

    public static int ANGLE_EJECT = 1960;
    public static int ANGLE_HANG = 1600;
    public static int ANGLE_SWALLOW = 1390;
    public static int ANGLE_TRAVEL = 1490; //safe to travel through backstage door
    public static double TIME_SWALLOW = 1;
    public static double TIME_EJECT = .5;
    public boolean leftRumbled = false;
    public boolean rightRumbled = false;

    //CONSTANTS
    HardwareMap hardwareMap;
    Robot robot;
    Servo diverterRight, diverterLeft;
    Servo angleLeft, angleRight;
    DcMotorEx beater;
    DistanceSensor pixelSensorRight, pixelSensorLeft;
    public static boolean precisionAngle = false;
    public boolean manualBeaterEject = false;
    public boolean manualBeaterEnable = false;
    public static double BEATER_INGEST_VELOCITY = 1700;

    public static double BEATER_SWALLOW_VELOCITY = 300;
    public static double BEATER_EJECT_VELOCITY = -700;
    public static double BEATER_SETTLE_EJECT_VELOCITY = -400;

    public static double beaterTargetVelocity = 0;

    public static int angleTarget = ANGLE_GROUND;
    private int ingestPixelHeight = 0;  //the height at which to start ingesting pixels. Normally 0 for ground but could be 4 for top pixel in a stack
    private int ingestStage = 0;
    private long ingestTimer = 0;

    public int getIngestPixelHeight() {
        return ingestPixelHeight;
    }

    public void setIngestPixelHeight(int ingestPixelHeight) {
        this.ingestPixelHeight = ingestPixelHeight < 0 ? 0 : ingestPixelHeight;
    }

    public void cleanArticulations() {
        swallowStage = 0;
        ejectState = 0;

    }

    public enum PixelStack {
        GROUND(0, ANGLE_INGEST_GROUND + ANGLE_INGEST_INCREMENT), //the minus is to force it harder into the tiles //1940
        TWO(1, ANGLE_INGEST_GROUND - (ANGLE_INGEST_INCREMENT)-5), //2000
        THREE(2, ANGLE_INGEST_GROUND - (ANGLE_INGEST_INCREMENT * 2)), //1975
        FOUR(3, ANGLE_INGEST_GROUND - (ANGLE_INGEST_INCREMENT * 3)), //1940
        FIVE(4, ANGLE_INGEST_GROUND - (ANGLE_INGEST_INCREMENT * 4)-5); //1915
        //1880 is top pixel

        private final int value;
        private final int angle;

        PixelStack(int value, int angle) {
            this.value = value;
            this.angle = angle;
        }

        public int getValue() {
            return value;
        }

        public int getAngle() {
            return angle;
        }

        public static PixelStack getByIndex(int index) {
            return PixelStack.values()[index];
        }

    }

    public enum PixelSensor {
        NONE(0),
        LEFT(1),
        RIGHT(1),
        BOTH(2);
        private final int count;

        PixelSensor(int count) {
            this.count = count;
        }

        public int getCount() {
            return count;
        }

        public static PixelSensor getByIndex(int index) {
            return PixelSensor.values()[index];
        }

        public static PixelSensor clear() {
            return PixelSensor.NONE;
        }

        public PixelSensor assertLeft() {
            if (this.equals(PixelSensor.RIGHT) || this.equals(PixelSensor.BOTH))
                return PixelSensor.BOTH;
            else
                return PixelSensor.LEFT;
        }

        public PixelSensor assertRight() {
            if (this.equals(PixelSensor.LEFT) || this.equals(PixelSensor.BOTH))
                return PixelSensor.BOTH;
            else return PixelSensor.RIGHT;
        }

        //syntactic helpers
        public boolean isBoth() {
            return (this.equals(PixelSensor.BOTH));
        }

        public boolean isLeft() {
            return (this.equals(PixelSensor.LEFT));
        }

        public boolean isRight() {
            return (this.equals(PixelSensor.RIGHT));
        }

        public boolean isNone() {
            return (this.equals(PixelSensor.NONE));
        }
    }

    PixelSensor pixelSensor = PixelSensor.clear();

    public enum Articulation {
        TRAVEL,
        INGEST,
        EJECT,
        MANUAL,
        HANG,
        SWALLOW,
        SETTLE,
        DOWN,
        INIT

    }

    public enum DiverterState {
        DELIVER_LEFT,
        DELIVER_RIGHT,
        DELIVER_BOTH
    }

    private DiverterState diverterState;
    public Articulation articulation;
    public static int numPixelsInStack = 5;


    public Intake(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        articulation = Articulation.MANUAL;
        diverterState = DiverterState.DELIVER_BOTH;
        angleTarget = ANGLE_INGEST_GROUND;

        diverterLeft = hardwareMap.get(Servo.class, "diverterRight");
        diverterRight = hardwareMap.get(Servo.class, "diverterLeft");
        beater = hardwareMap.get(DcMotorEx.class, "intakeBeater");
        beater.setDirection(DcMotorSimple.Direction.REVERSE);
        angleLeft = hardwareMap.get(Servo.class, "intakeAngleLeft");
        angleRight = hardwareMap.get(Servo.class, "intakeAngleRight");
        pixelSensorRight = hardwareMap.get(DistanceSensor.class, "rightPixelSensor");
        pixelSensorLeft = hardwareMap.get(DistanceSensor.class, "leftPixelSensor");
        angleRight.setDirection(Servo.Direction.FORWARD);
        angleLeft.setDirection(Servo.Direction.REVERSE);
        beater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        pixelSensors();
        articulate();
        angleLeft.setPosition(Utils.servoNormalize(angleTarget));
        angleRight.setPosition(Utils.servoNormalize(angleTarget));
        diverters();
        beater.setVelocity(beaterTargetVelocity);
    }

    void pixelSensors() {
        //right now this is a virtual sensor operated by the drive team
        //and currently no implementation is required outside of DriverControls
        //this may change
    }

    public void pixelSensorLeft() {
        pixelSensor = pixelSensor.assertLeft();
    }

    public void pixelSensorRight() {
        pixelSensor = pixelSensor.assertRight();
    }

    public void pixelSensorClear() {

        pixelSensor = pixelSensor.clear();
    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        return articulation;
    }

    public Articulation articulate() {
        switch (articulation) {
            case TRAVEL: //intake should spend most of its time here
                //the angle is one that compacts the robot as much as possible and still
                //allows it to pass under the backstage door
                angleTarget = ANGLE_TRAVEL;
                //also reinforce that the beater should be still
                beaterTargetVelocity = 0;
                //also assert that manual beater override has been turned off
                manualBeaterEnable = false;
                break;
            case MANUAL: //todo still need to refactor manual?
                if (manualBeaterEnable) {
                    if (!manualBeaterEject) {
                        beater.setPower(1);
                        beaterTargetVelocity = BEATER_INGEST_VELOCITY;
                    } else {
                        beater.setPower(1);
                        beaterTargetVelocity = BEATER_EJECT_VELOCITY;
                    }
                } else { //manual override was disabled externally, return to Travel
                    manualBeaterEject = false;
                    articulation = Articulation.TRAVEL;
                }
                break;
            case SWALLOW:
                if (swallow())
                    articulation = Articulation.TRAVEL;
                break;
            case SETTLE:
                if (settle())
                    articulation = Articulation.TRAVEL;
                break;
            case HANG:
                angleTarget = ANGLE_HANG;
                manualBeaterEject = false;
                manualBeaterEnable = false;
                articulation = Articulation.MANUAL;
                break;
            case INGEST:
                if (ingest(ingestPixelHeight)) {
                    articulation = Articulation.SWALLOW;
                }
                break;
            case EJECT:
                if (eject()) {
                    articulation = Articulation.TRAVEL;
                }
                break;
            case DOWN:
                angleTarget = ANGLE_EJECT;
                break;
            case INIT:
                angleTarget = ANGLE_START;
                articulation = Articulation.MANUAL;
                break;
        }
        return articulation;
    }

    public boolean isEating() {
        if (articulation.equals(Articulation.INGEST) || articulation.equals(Articulation.SWALLOW))
            return true;
        else return false;
    }

    public void setAngle(int pwm) {
        angleTarget = pwm;
        if (angleTarget < ANGLE_PWM_MIN) {
            angleTarget = ANGLE_PWM_MIN;
        }
        if (angleTarget > ANGLE_PWM_MAX) {
            angleTarget = ANGLE_PWM_MAX;
        }
    }

    public void setDiverters(DiverterState diverterState) {
        this.diverterState = diverterState;
    }

    public void setDiverters(PixelSensor ps, int height) {
        diverterState = DiverterState.DELIVER_BOTH;

        if (height == 0) {
            if (ps.isLeft()) diverterState = DiverterState.DELIVER_RIGHT;
            else if (ps.isRight()) diverterState = DiverterState.DELIVER_LEFT;
        } else {
            if (ps.isNone())
                diverterState = DiverterState.DELIVER_LEFT;
            else if (ps.isLeft()) diverterState = DiverterState.DELIVER_RIGHT;
            else if (ps.isRight()) diverterState = DiverterState.DELIVER_LEFT;
        }
        //default to both

    }

    public DiverterState getDiverters() {
        return diverterState;
    }

    public boolean diverters() {
        switch (diverterState) {
            case DELIVER_BOTH:
                diverterRight.setPosition(Utils.servoNormalize(RIGHT_DIVERTER_OPEN));
                diverterLeft.setPosition(Utils.servoNormalize(LEFT_DIVERTER_OPEN));
                break;
            case DELIVER_RIGHT:
                diverterRight.setPosition(Utils.servoNormalize(RIGHT_DIVERTER_OPEN));
                diverterLeft.setPosition(Utils.servoNormalize(LEFT_DIVERTER_CLOSED));
                break;
            case DELIVER_LEFT:
                diverterRight.setPosition(Utils.servoNormalize(RIGHT_DIVERTER_CLOSED));
                diverterLeft.setPosition(Utils.servoNormalize(LEFT_DIVERTER_OPEN));
                break;
        }
        return true;
    }

    public boolean ingest(int height) { //height is expected to be changed externally
        setDiverters(pixelSensor, height);
        angleTarget = PixelStack.getByIndex(Range.clip(height - pixelSensor.count, 0, 4)).angle;
        switch (ingestStage) {
            case 0:
                beaterTargetVelocity = BEATER_INGEST_VELOCITY;
                ingestTimer = futureTime(3);
                ingestStage++;
            case 1:
                if (isPast(ingestTimer)) {
                    Sensors.pixelSensorEnabled = true;
                    if (Robot.sensors.leftPixelSensorValue < 1) {
                        if(!leftRumbled) {
                            IntoTheDeep_6832.dc.rumble(1, 500);
                            leftRumbled = true;
                        }
                        pixelSensorLeft();
                    }
                    if (Robot.sensors.rightPixelSensorValue < 1) {
                        if(!rightRumbled) {
                            IntoTheDeep_6832.dc.rumble(1, 500);
                            rightRumbled = true;
                        }
                        pixelSensorRight();
                    }
                    //if a pixel is detected
                    if (pixelSensor.count > 1) {
                        rightRumbled = false;
                        leftRumbled = false;
                        IntoTheDeep_6832.dc.rumble(1, 2000);
                        Sensors.pixelSensorEnabled = false;
                        pixelSensorClear();
                        ingestStage = 0;
                        return true;
                    }
                }
        }
        return false;
    }

    int swallowStage = 0;
    long swallowTimer;

    public boolean swallow() {
        switch (swallowStage) {
            case 0://todo swallow timing values have not been validated
                //todo, add some safety checking, like is the scoopagon docked?
                pixelSensorClear();
                diverterState = DiverterState.DELIVER_BOTH;
//                setAngle(ANGLE_SWALLOW);
                angleTarget = ANGLE_SWALLOW;
                //todo are these needed? clunky way to allow override
                manualBeaterEnable = false;
                manualBeaterEject = false;
                beaterTargetVelocity = BEATER_SWALLOW_VELOCITY;
                swallowTimer = futureTime(TIME_SWALLOW);
                swallowStage++;
                break;
            case 1:
                if (isPast(swallowTimer))
                    swallowStage++;
                break;
            case 2:
                beaterTargetVelocity = 0; //stop beater
                swallowStage = 0;
                //don't need to set next angle since that's the job of the state this resolves to
                return true;
        }
        return false;
    }

    int settleStage = 0;
    long settleTimer;
    int settleRepeats = 0;

    public boolean settle() {
        switch (settleStage) {
            case 0:
                pixelSensorClear();
                diverterState = DiverterState.DELIVER_BOTH;
                angleTarget = ANGLE_SWALLOW;
                if (settleRepeats == 0)
                    settleRepeats = 1;
                manualBeaterEnable = false;
                manualBeaterEject = false;
                beaterTargetVelocity = BEATER_SWALLOW_VELOCITY;
                settleTimer = futureTime(TIME_SWALLOW);
                settleStage++;
                break;
            case 1:
                if (isPast(settleTimer))
                    settleStage++;
                break;
            case 2:
                beaterTargetVelocity = 0;
                settleStage++;
                break;
            case 3:
                beaterTargetVelocity = BEATER_SETTLE_EJECT_VELOCITY;
                settleTimer = futureTime(.5);
                settleStage++;
                break;
            case 4:
                if (isPast(settleTimer)) {
                    beaterTargetVelocity = 0;
                    settleStage = 0;
                    settleRepeats--;
                    if (settleRepeats == 0)
                        return true;
                }
                break;
        }
        return false;
    }

    public boolean readyForTravel() {
        return articulation.equals(Articulation.TRAVEL);
    }

    int ejectState = 0;
    double ejectTimer = TIME_EJECT;

    public boolean eject() {
        switch (ejectState) {
            case 0: //todo timing values in eject() have not been validated
                angleTarget = ANGLE_EJECT;
                ejectTimer = futureTime(.5); //time for angle to set
                ejectState++;
                break;
            case 1:
                if (isPast(ejectTimer)) {
                    beaterTargetVelocity = BEATER_EJECT_VELOCITY;
                    ejectTimer = futureTime(TIME_EJECT);
                    ejectState++;
                }
                break;
            case 2:
                if (isPast(ejectTimer)) {
                    beaterTargetVelocity = 0;
                    ejectState = 0;
                    articulation = Articulation.INIT;
                    return true;
                }
                break;
        }
        return false;
    }

    public void togglePrecisionAngle() {
        precisionAngle = !precisionAngle;
    }

    public double adjustAngle(double speed) {
        angleTarget += speed * (precisionAngle ? 10 : 100);
        return angleTarget;
    }


    public void toggleBeaterEnable() {
        manualBeaterEnable = !manualBeaterEnable;
    }

    public void toggleBeaterDirection() {
        manualBeaterEject = !manualBeaterEject;
    }

    @Override
    public void stop() {
        beater.setVelocity(0);
    }

    @Override
    public void resetStates() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("pixelsensor state", pixelSensor.name());
        if (Sensors.pixelSensorEnabled) {
            telemetryMap.put("leftPixelSensor", Robot.sensors.leftPixelSensorValue);
            telemetryMap.put("rightPixelSensor", Robot.sensors.rightPixelSensorValue);
        }
        telemetryMap.put("ingest pixel height", ingestPixelHeight);
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("manual beater bar on?", manualBeaterEnable);
        telemetryMap.put("beater bar amps", Robot.sensors.beaterBarAmps);
        telemetryMap.put("beater bar velocity", beater.getVelocity());
        telemetryMap.put("beater bar target velocity", beaterTargetVelocity);
        telemetryMap.put("angle controller position left", Utils.servoDenormalize(angleLeft.getPosition()) + " & right" + Utils.servoDenormalize(angleRight.getPosition()));
        telemetryMap.put("diverter state", diverterState.name());
        telemetryMap.put("right diverter ticks", Utils.servoDenormalize(diverterRight.getPosition()));
        telemetryMap.put("left diverter ticks", Utils.servoDenormalize(diverterLeft.getPosition()));
        telemetryMap.put("intake target angle", angleTarget);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "INTAKE";
    }
}
