//package org.firstinspires.ftc.teamcode.robots.swervolicious;
//
//import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
//import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.field;
//import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
//import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
//import static org.firstinspires.ftc.teamcode.util.utilMethods.wrapAngle;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.robots.deepthought.field.POI;
//import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.Robot;
//import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.Trident;
//import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.samplers.Sampler;
//import org.firstinspires.ftc.teamcode.robots.deepthought.util.DTPosition;
//import org.firstinspires.ftc.teamcode.robots.deepthought.util.TelemetryProvider;
//
//import java.util.LinkedHashMap;
//import java.util.Map;
//
//@Config(value = "AA_ITD_Auto_Spec")
//public class AutoSpecimens implements TelemetryProvider {
//    public static double AUTON_WAIT_TIMER = 1;
//    public static int numCycles = 4;
//    private Robot robot;
//    private HardwareMap hardwareMap;
//
//    //
//    public enum AutonState {
//        INIT, DRIVE_TO_HIGHBAR, DRIVE_TO_OZONE, OUTTAKE_TO_HIGHBAR, DRIVE_TO_SUB,
//    }
//
//    public AutonState autonState = AutonState.INIT;
//
//    @Override
//    public Map<String, Object> getTelemetry(boolean debug) {
//        Map<String, Object> telemetryMap = new LinkedHashMap<>();
//        telemetryMap.put("autonState\t ", autonState);
//        telemetryMap.put("autonIndex\t", autonIndex);
//        telemetryMap.put("autonSweepIndex", autonSweepIndex);
//        telemetryMap.put("driveAndLatchIndex\t", driveAndLatchIndex);
//        telemetryMap.put("driveandWalltakeIndex\t", driveAndWalltakeIndex);
//        return telemetryMap;
//    }
//
//    @Override
//    public String getTelemetryName() {
//        return "AUTOSPEC";
//    }
//
//    // autonomous routines
//
//    public static int selectedPath;
//    public int allianceMultiplier = 1;
//
//
//    public static double FIELD_INCHES_PER_GRID = 23.5;
//    public static double AUTON_START_DELAY = 0;
//
//    public AutoSpecimens(Robot robot) {
//        this.robot = robot;
//        this.hardwareMap = this.robot.hardwareMap;
//        autonIndex = 0;
//    }
//
//    public static int autonIndex;
//    public long autonTimer = futureTime(10);
//    public long gameTimer;
//
//    public boolean execute(TelemetryPacket packet) {
//        if (!alliance.isRed()) {
//            allianceMultiplier = -1;
//        }
//        robot.positionCache.update(new DTPosition(robot.driveTrain.localizer.getPose(), robot.trident.getShoulderCurrentPosition(), robot.trident.sampler.slide.getCurrentPosition(), robot.trident.speciMiner.slide.getCurrentPosition()), false);
//        switch (autonIndex) { //auton delay
//            case 0:
//                gameTimer = futureTime(27);
//                autonState = AutonState.INIT;
//                autonTimer = futureTime(AUTON_START_DELAY);
//                autonIndex++;
//                break;
//            case 1: // travel to hibar field position
//                autonState = AutonState.DRIVE_TO_HIGHBAR; // drive to sub
//                if (isPast(autonTimer)) {
//                    if (robot.driveTrain.strafeToPose(field.hibar.getPose(), packet)) {
//                        autonIndex++;
//                    }
//                }
//                break;
//            case 2: // set pre latch arm position todo start near very end of drive
//                robot.trident.speciMiner.prelatchHigh();  // preset arm position
//                autonIndex++;
//
//                break;
//
//            case 3: // reserve in case we need to micro adjust field position before latching
//                autonTimer = futureTime(1); //enough time to complete latch
//                autonIndex++;
//                break;
//            case 4: // latch specimen
//                if (robot.trident.speciMiner.latch()) {
//                    robot.resetStates();
//                    robot.articulate(Robot.Articulation.MANUAL);
//                    autonIndex++;
//                }
//                break;
//            case 5: // eject - might not be needed?
//                if (robot.trident.speciMiner.eject()) if (robot.trident.tuck()) autonIndex++;
//                break;
//
//            case 6: // back up a bit? in case strafe conflicts with sub
////                return true;
//                autonIndex++;
//                break;
//            case 7: // start sweeping the ground samples
//                if (autonShiftSample(field.ground4, packet)) {
//                    autonIndex++;
//                    return true;
//                }
//                break;
//            case 8:
//                if (autonShiftSample(field.ground5, packet)) {
//                    autonIndex++;
//                }
//                break;
//            case 9:
//                if (autonShiftSample(field.ground6, packet)) {
//                    autonIndex++;
//                }
//                break;
//            case 10:
//                autonIndex = 0;
//                return true;
//        }
//        return false;
//    }
//
//    public boolean execSweeping(TelemetryPacket packet) {
//        if (!alliance.isRed()) {
//            allianceMultiplier = -1;
//        }
//        robot.positionCache.update(new DTPosition(robot.driveTrain.localizer.getPose(), robot.trident.getShoulderCurrentPosition(), robot.trident.sampler.slide.getCurrentPosition(), robot.trident.speciMiner.slide.getCurrentPosition()), false);
//        switch (autonIndex) { //auton delay
//            case 0:
//                resetStates(); // resets all state variables
//                gameTimer = futureTime(27);
//                autonState = AutonState.INIT;
//                autonTimer = futureTime(AUTON_START_DELAY);
//                autonIndex = 7; //skip over preload
//                break;
//            case 1: // drive to hibar with gripped specimen and latch
//                autonState = AutonState.DRIVE_TO_HIGHBAR; // drive to sub
//                if (isPast(autonTimer)) {
//                    if (driveAndLatch(packet, 0)) {
//                        //return true;
//                        autonIndex = 7;
//                    }
//                }
//                break;
//            case 7: // start sweeping the ground samples
//                if (autonSweepSample(field.sweep1, field.sweep1Oz, false, packet)) {
////                    robot.aprilTagRelocalization(false);
//                    autonIndex+=3;
//                }
//                break;
//            case 8:
//                if (autonSweepSample(field.sweep2, field.sweep2Oz, false, packet)) {
//                    autonIndex++;
//                }
//                break;
//            case 9:
//                if (autonSweepSample(field.sweep3, field.sweep3Oz, false, packet)) {
//                    autonIndex++;
//                }
//                break;
//            case 10:
//                if (driveAndWalltake(packet)) {
//                    resetStates();
//                    robot.resetStates();
//                    robot.trident.speciMiner.setIntaking();
//                    autonIndex++;
//                }
//                break;
//            case 11:
//                if (driveAndLatch(packet, 1)) {
//                    robot.resetStates();
//                    autonIndex++;
//                }
//                break;
//            case 12:
//                if (driveAndWalltake(packet)) {
//                    resetStates();
//                    robot.resetStates();
//                    robot.trident.speciMiner.setIntaking();
//                    autonIndex++;
//                }
//                break;
//            case 13:
//                if (driveAndLatch(packet, 2)) {
//                    robot.resetStates();
//                    autonIndex++;
//                    autonTimer = futureTime(2);
//                }
//                break;
//            case 14:
//                driveAndWalltake(packet);
//                if (driveAndWalltakeIndex > 1 || isPast(autonTimer)) {
//                    resetStates();
//                    robot.resetStates();
//                    robot.articulate(Robot.Articulation.TRAVEL);
//                    autonIndex++;
//                }
//
//
//                break;
//            case 15:
//                resetStates();
//                return true;
//
//
//        }
//        return false;
//    }
//
//    //includes driving to outtake, actual latching, and leaves the robot in outtake position
//    public int autonOuttakeIndex = 0;
//    public long autonOuttakeTimer = 0;
//
//    public boolean autonSpecimenOuttake(TelemetryPacket packet) {
//        switch (autonOuttakeIndex) {
//            case 0: // not sure we need another wait here if there is one in execute()
//                robot.resetStates();
//                autonOuttakeTimer = futureTime(AUTON_WAIT_TIMER);
//                autonOuttakeIndex++;
//                break;
//            case 1: // score the preload alliance sample
//                if (isPast(autonOuttakeTimer)) {
//                    Trident.enforceSlideLimits = true;
//                    robot.articulate(Robot.Articulation.SPECIMINER_OUTTAKE);
//                    autonOuttakeTimer = futureTime(1.75);
//                    //todo - set Speciminer and Shoulder for hibar prep while driving
//
//                }
//                // drive to hibar prep location
//                if (robot.driveTrain.strafeToPose(field.hibarPrep.getPose(), packet)) {
//                    autonOuttakeIndex++;
//                }
//                break;
//            case 2: //todo keep modifying for specimens
//                if (isPast(autonOuttakeTimer)) {
////                    robot.aprilTagRelocalization();
//                    autonOuttakeTimer = futureTime(2);
//                    autonOuttakeIndex++;
//                }
//                break;
//
//            case 3:
//                robot.aprilTagRelocalization(true);
//                if (isPast(autonOuttakeTimer)) {
//                    robot.trident.sampler.servoPower = 0;
//                    autonOuttakeIndex = 0;
//                    return true;
//                }
//                break;
//        }
//        return false;
//    }
//
//    // drive to hibar and latch specimen
//    // only safe to start from a field position that can direct travel to hibar
//    int driveAndLatchIndex = 0;
//    double driveAndLatchTimer = 0;
//
//    boolean driveAndLatch(TelemetryPacket packet, int num) {
//        switch (driveAndLatchIndex) { //auton delay
//            case 0:
//                resetStates(); // resets all state variables
//                driveAndLatchIndex++;
//                break;
//            case 1: // travel to hibar field position
//
//                autonState = AutonState.DRIVE_TO_HIGHBAR; // drive to sub
//                robot.trident.speciMiner.prelatchHighSlide(); //slide is slow, start extending
//                if (field.hibar.distTo(robot.driveTrain.localizer.getPose()) < 1) { //trigger shoulder on reaching within 1 field tile of highbar position
//                    robot.trident.sampler.setElbowAngle(Sampler.SWEEP_ELBOW_ANGLE);
//                    robot.trident.speciMiner.prelatchHigh();  // preset arm positions}
//                }
//                if (robot.driveTrain.strafeToPose(field.hibar.getPose(), packet)) {
////                    driveAndLatchTimer = futureTime(5);
//                    robot.trident.sampler.setElbowAngle(Sampler.SWEEP_ELBOW_ANGLE);
//                    driveAndLatchIndex++;
//                    robot.trident.speciMiner.setResting();
//                }
//                break;
//            case 2: // set pre latch arm position todo start near very end of drive
//                robot.trident.sampler.setElbowAngle(Sampler.SWEEP_ELBOW_ANGLE);
//                robot.trident.speciMiner.prelatchHigh();  // preset arm positions
//                driveAndLatchIndex++;
//
//                break;
//
//
//            case 3: // reserve in case we need to micro adjust field position before latching
////                driveAndLatchTimer = futureTime(1);
//                driveAndLatchIndex++;
//                break;
//            case 4: // latch specimen
//                if (robot.trident.speciMiner.latch()) {
//                    driveAndLatchIndex++;
//                }
//                break;
//            case 5: // eject - might not be needed?
////                if (robot.trident.speciMiner.eject())
//                driveAndLatchIndex++;
//                //if (robot.trident.tuck())
//                break;
//
//            case 6: // back up a bit? in case strafe conflicts with sub
//                robot.trident.speciMiner.setEjecting();
//                resetStates();
//                driveAndLatchIndex = 0;
//                return true;
//        }
//        return false;
//    }
//
//    // drive to wall and retrieve specimen
//    // only safe to start from a field position that can direct travel to wall pickup
//    int driveAndWalltakeIndex = 0;
//    double driveAndWalltakeTimer = 0;
//
//    boolean driveAndWalltake(TelemetryPacket packet) {
//        switch (driveAndWalltakeIndex) { //auton delay
//            case 0:
////                resetStates(); // resets all state variables
//                driveAndWalltakeIndex++;
//                break;
//            case 1: // travel to walltake field position
//                autonState = AutonState.DRIVE_TO_OZONE; // drive to ozone pickup location
////                if (isPast(driveAndWalltakeTimer)) {
//                robot.trident.speciMiner.wallTakePresets(); //configure Speciminer to intake from wall
//                if (robot.driveTrain.strafeToPose(field.oZoneWalltake.getPose(), packet)) {
//                    robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(.15, 0), 0));
//                    robot.trident.speciMiner.resetStates();
//                    driveAndWalltakeIndex++;
//                    driveAndWalltakeTimer = futureTime(2.5);
//                }
////                }
//                break;
//            case 2: // start testing intake - stops on color sensor detection - todo start near very end of drive
////                robot.aprilTagRelocalization(false);
////                if (robot.trident.speciMiner.wallTake(true)) {
////                    robot.driveTrain.drive(0, 0, 0); // stop chassis driving
////                    driveAndWalltakeIndex = 0;
////                    return true;
////                }
//                robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(.15, 0), 0));
//                if (Math.abs(robot.driveTrain.localizer.getPose().position.y) / FIELD_INCHES_PER_GRID > 2.3  || isPast(driveAndWalltakeTimer)) {
//                    driveAndWalltakeTimer = futureTime(.2);
//                    robot.trident.setShoulderTarget(robot.trident.speciMiner, robot.trident.getShoulderTarget() + 200);
//                    robot.driveTrain.drive(0, 0, 0); // stop chassis drivings
//                    driveAndWalltakeIndex++;
//                }
//                break;
//            case 3:
//                if (isPast(driveAndWalltakeTimer)) {
//                    driveAndWalltakeIndex = 0;
//                    return true;
//                }
//                break;
//
//        }
//        return false;
//    }
//
//    public int autonShiftIndex = 0;
//    public int autonShiftTimer = 0;
//    int numAttempts = 2;
//
//    //shift a given sample to ozone using the chassis backplate
//    public boolean autonShiftSample(POI ground, TelemetryPacket packet) {
////        robot.aprilTagRelocalization();
//        switch (autonShiftIndex) {
//            case 0: // drive from hibar or from starting position to safe intermediate
//                if (robot.driveTrain.strafeToPose(field.zig.getPose(), packet)) {
//                    autonShiftIndex++;
//                    //robot.resetStates();
//                }
//            case 1: // get beyond alliance samples
//                if (robot.driveTrain.strafeToPose(field.zag.getPose(), packet)) {
//                    autonShiftIndex++;
//                    //robot.resetStates();
//                }
//                break;
//            case 2: //drive beyond target sample
//                if (robot.driveTrain.strafeToPose(ground.getPose(), packet)) {
//                    autonShiftIndex++;
//                    //robot.resetStates();
//                }
//                break;
//            case 3: // push to ozone - current X wih a y displacement
//                if (robot.driveTrain.strafeToPose(new Pose2d(ground.getPose().position.x, -2.3, 90), packet)) {
//                    autonShiftIndex++;
//                    //robot.resetStates();
//                    robot.articulate(Robot.Articulation.TRAVEL);
//                    autonShiftIndex = 0;
//                    return true;
//                }
//                break;
//        }
//        return false;
//    }
//
//    public int autonSweepIndex = 0;
//    public int autonSweepTimer = 0;
//
//    //Sweep a given sample to ozone using the Sampler
//    public boolean autonSweepSample(POI sweepFrom, POI ozone, boolean recover, TelemetryPacket packet) {
//        switch (autonSweepIndex) {
//            case 0: // set sampler for sweeping over
//                //if(robot.trident.sampler.sweepConfig(true))
//                robot.trident.sampler.sweepConfig(false);
//                autonSweepIndex++;
//                break;
//            case 1: // drive to sweeping start position
//                if (robot.driveTrain.strafeAndTurn(sweepFrom.getPose(), packet)) {
//                    autonSweepIndex++;
//                }
//                break;
//            case 2: // sweep setting
//                if (robot.trident.sampler.sweepConfig(false)) // sampler floats just above floor
//                    autonSweepIndex++;
//                break;
//            case 3: // let's sweep
//                //if (robot.driveTrain.strafeToPose(ozone.getPose(), packet)) { //strafeToPose is slow
//                if (robot.driveTrain.turnUntilDegreesIMU(wrapAngle(ozone.heading), .5 )) {
//                    robot.trident.sampler.sweepConfig(true); //set for sweepOver return
//                    autonSweepIndex++;
//                }
//                break;
//            case 4: // sweepOver back to beginning position
//                if (recover) { // can skip recovery for last floor sample
//                    //if (robot.driveTrain.strafeToPose(sweepFrom.getPose(), packet)) {
//                    if (robot.driveTrain.turnUntilDegreesIMU(wrapAngle(sweepFrom.heading), .6)) {
//                        resetStates(); //be careful with this
//                        return true;
//                    }
//                } else {
//                    resetStates(); //be careful with this
//                    return true;
//                }
//                break;
//        }
//        return false;
//    }
//
//    void resetStates() {
//        //do NOT reset autonIndex
//        driveAndLatchIndex = 0;
//        autonSweepIndex = 0;
//        autonShiftIndex = 0;
//        //do not call robot.resetStates - creates a loop since it calls all subsystem resets
//        //robot.resetStates();
//    }
//}
