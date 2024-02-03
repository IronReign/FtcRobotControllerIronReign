package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "AA_CS_INTAKE")
public class Intake implements Subsystem {
    public static int RIGHT_DIVERTER_OPEN = 1010;
    public static int LEFT_DIVERTER_OPEN = 1850;
    public static int LEFT_DIVERTER_CLOSED = 1340;
    public static int RIGHT_DIVERTER_CLOSED = 1500;
    public static int ANGLE_GROUND = 1325; //where the intake hits the ground
    public static int ANGLE_INGEST_INCREMENT = 20;
    public static int ANGLE_MIN = ANGLE_GROUND - ANGLE_INGEST_INCREMENT;
    public static int ANGLE_MAX = ANGLE_GROUND + 835;
    public static int ANGLE_START = ANGLE_MAX;
    public static int ANGLE_INGEST_GROUND = ANGLE_GROUND;

    public static int ANGLE_EJECT = ANGLE_GROUND + 100;
    public static int ANGLE_HANG = ANGLE_GROUND + 200; //1515+ INTAKE_OFFSET;
    public static int ANGLE_SWALLOW = ANGLE_GROUND + 525;//1810 + INTAKE_OFFSET;
    public static int ANGLE_TRAVEL = ANGLE_GROUND + 335;//1750+ INTAKE_OFFSET; //safe to travel through backstage door
    public static double TIME_SWALLOW = 1;
    public static double TIME_EJECT = .5;

    //CONSTANTS
    HardwareMap hardwareMap;
    Robot robot;
    Servo diverterRight, diverterLeft;
    Servo angle;
    DcMotorEx beater;
    public static boolean precisionAngle = false;
    public boolean manualBeaterEject = false;
    public boolean manualBeaterEnable = false;
    public static double BEATER_INGEST_VELOCITY = 1700;
    public static double BEATER_EJECT_VELOCITY = -700;

    private double beaterTargetVelocity = 0;

    private static int angleTarget = ANGLE_GROUND;
    private int ingestPixelHeight = 0;  //the height at which to start ingesting pixels. Normally 0 for ground but could be 4 for top pixel in a stack

    public int getIngestPixelHeight() {
        return ingestPixelHeight;
    }

    public void setIngestPixelHeight(int ingestPixelHeight) {
        this.ingestPixelHeight = ingestPixelHeight < 0? 0 : ingestPixelHeight;
    }

    public void cleanArticulations() {
        swallowStage = 0;
        ejectState = 0;

    }

    public enum PixelStack {
        GROUND(0, ANGLE_INGEST_GROUND - ANGLE_INGEST_INCREMENT), //the minus is to force it harder into the tiles
        TWO(1, ANGLE_INGEST_GROUND + ANGLE_INGEST_INCREMENT - 15),
        THREE(2, ANGLE_INGEST_GROUND + ANGLE_INGEST_INCREMENT * 2 - 15),
        FOUR(3, ANGLE_INGEST_GROUND + ANGLE_INGEST_INCREMENT * 3 - 15),
        FIVE(4, ANGLE_INGEST_GROUND + ANGLE_INGEST_INCREMENT * 4 - 15);

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
        public static PixelSensor clear(){
            return PixelSensor.NONE;
        }

        public PixelSensor assertLeft(){
            if (this.equals(PixelSensor.RIGHT) || this.equals(PixelSensor.BOTH))
                return PixelSensor.BOTH;
            else
                return PixelSensor.LEFT;
        }
        public PixelSensor assertRight(){
            if (this.equals(PixelSensor.LEFT) || this.equals(PixelSensor.BOTH))
                return PixelSensor.BOTH;
            else return PixelSensor.RIGHT;
        }
        //syntactic helpers
        public boolean isBoth() {return (this.equals(PixelSensor.BOTH));}
        public boolean isLeft() {return (this.equals(PixelSensor.LEFT));}
        public boolean isRight() {return (this.equals(PixelSensor.RIGHT));}
        public boolean isNone() {return (this.equals(PixelSensor.NONE));}
    }
    PixelSensor pixelSensor = PixelSensor.clear();

    public enum Articulation {
        TRAVEL,
        INGEST,
        EJECT,
        MANUAL,
        HANG,
        SWALLOW,
        DOWN,
        INIT

    }

    public enum DiverterState{
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
            angle = hardwareMap.get(Servo.class, "intakeAngle");

            beater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        pixelSensors();
        articulate();
        angle.setPosition(Utils.servoNormalize(angleTarget));
        diverters();
        beater.setVelocity(beaterTargetVelocity);
    }

    void pixelSensors(){
        //right now this is a virtual sensor operated by the drive team
        //and currently no implementation is required outside of DriverControls
        //this may change
    }
    public void pixelSensorLeft(){
        pixelSensor = pixelSensor.assertLeft();
    }
    public void pixelSensorRight(){
        pixelSensor = pixelSensor.assertRight();
    }
    public void pixelSensorClear(){
        pixelSensor = pixelSensor.clear();
    }

    public Articulation articulate(Articulation target) {
        articulation = target;
        return articulation;
    }

    public Articulation articulate(){
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
                if(manualBeaterEnable) {
                    if(!manualBeaterEject) {
                        beater.setPower(1);
                        beaterTargetVelocity = BEATER_INGEST_VELOCITY;
                    }
                    else {
                        beater.setPower(1);
                        beaterTargetVelocity= BEATER_EJECT_VELOCITY;
                    }
                }
                else { //manual override was disabled externally, return to Travel
                    manualBeaterEject=false;
                    articulation = Articulation.TRAVEL;
                }
                break;
            case SWALLOW:
                if (swallow())
                    articulation = Articulation.TRAVEL;
                break;
            case HANG:
                angleTarget = ANGLE_HANG;
                manualBeaterEject = false;
                manualBeaterEnable = false;
                articulation = Articulation.MANUAL;
                break;
            case INGEST:
                if(ingest(ingestPixelHeight)) {
                    articulation = Articulation.SWALLOW;
                }
                break;
            case EJECT:
                if(eject()) {
                    articulation = Articulation.TRAVEL;
                }
                break;
            case DOWN:
                angleTarget = ANGLE_EJECT;
                angle.setPosition(Utils.servoNormalize(angleTarget));
                break;
            case INIT:
                angleTarget = ANGLE_START;
                angle.setPosition(Utils.servoNormalize(angleTarget));
                articulation = Articulation.MANUAL;
                break;
        }
        return articulation;
    }

    public boolean isEating(){
        if (articulation.equals(Articulation.INGEST) || articulation.equals(Articulation.SWALLOW))
            return true;
        else return false;
    }

    public void setAngle(int pwm) {
        angleTarget = pwm;
        if(angleTarget < ANGLE_MIN) {
            angleTarget = ANGLE_MIN;
        }
        if(angleTarget > ANGLE_MAX) {
            angleTarget = ANGLE_MAX;
        }
    }

    public void setDiverters(DiverterState diverterState){
        this.diverterState = diverterState;
    }
    public void setDiverters(PixelSensor ps, int height){
        diverterState = DiverterState.DELIVER_BOTH;

        if(height == 0) {
            if (ps.isLeft()) diverterState = DiverterState.DELIVER_RIGHT;
            else if (ps.isRight()) diverterState = DiverterState.DELIVER_LEFT;
        }
        else {
            if (ps.isNone())
                diverterState = DiverterState.DELIVER_LEFT;
            else if (ps.isLeft()) diverterState = DiverterState.DELIVER_RIGHT;
            else if (ps.isRight()) diverterState = DiverterState.DELIVER_LEFT;
        }
        //default to both

    }
    public DiverterState getDiverters(){
        return diverterState;
    }
    public boolean diverters(){
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

    public boolean ingest(int height){ //height is expected to be changed externally
        setDiverters(pixelSensor, height);
        angleTarget = PixelStack.getByIndex(Range.clip(height-pixelSensor.count,0,4)).angle;
        beaterTargetVelocity = BEATER_INGEST_VELOCITY;
        //if a pixel is detected
        if(pixelSensor.count>1) return true;
        return false;
    }
    int swallowStage = 0;
    long swallowTimer;

    public boolean swallow() {
        switch (swallowStage){
            case 0://todo swallow timing values have not been validated
                //todo, add some safety checking, like is the scoopagon docked?
                pixelSensorClear();
                diverterState = DiverterState.DELIVER_BOTH;
                setAngle(ANGLE_SWALLOW);
                //todo are these needed? clunky way to allow override
                manualBeaterEnable = false;
                manualBeaterEject = false;
                beaterTargetVelocity = BEATER_INGEST_VELOCITY;
                swallowTimer = futureTime(TIME_SWALLOW);
                swallowStage++;
                break;
            case 1:
                if (isPast(swallowTimer))
                    swallowStage++;
                break;
            case 2:
                beaterTargetVelocity = 0; //stop beater
                swallowStage=0;
                //don't need to set next angle since that's the job of the state this resolves to
                return true;
        }
        return false;
    }

    public boolean readyForTravel() {
        return articulation.equals(Articulation.TRAVEL);
    }

    int ejectState = 0;
    double ejectTimer = TIME_EJECT;
    public boolean eject() {
        switch (ejectState){
            case 0: //todo timing values in eject() have not been validated
                setAngle(ANGLE_EJECT);
                ejectTimer = futureTime(.3); //time for angle to set
                ejectState++;
                break;
            case 1:
                if (isPast(ejectTimer)){
                    beaterTargetVelocity = BEATER_EJECT_VELOCITY;
                    ejectTimer = futureTime(TIME_EJECT);
                    ejectState++;
                }
                break;
            case 2:
                if (isPast(ejectTimer)){
                    beaterTargetVelocity = 0;
                    ejectState = 0;
                    articulation = Articulation.TRAVEL;
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


        angle.setPosition(
        Utils.servoNormalize(angleTarget));
        return angleTarget;
    }


    public void toggleBeaterEnable() {
        manualBeaterEnable = !manualBeaterEnable;
    }
    public void toggleBeaterDirection(){
        manualBeaterEject = !manualBeaterEject;
    }

    @Override
    public void stop() {
        beater.setVelocity(0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("pixelsensor state", pixelSensor.name());
        telemetryMap.put("ingest pixel height", ingestPixelHeight);
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("manual beater bar on?", manualBeaterEnable);
        telemetryMap.put("beater bar amps", beater.getPower());
        telemetryMap.put("beater bar velocity", beater.getVelocity());
        telemetryMap.put("beater bar target velocity", beaterTargetVelocity);
        telemetryMap.put("angle controller position", Utils.servoDenormalize(angle.getPosition()));
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
