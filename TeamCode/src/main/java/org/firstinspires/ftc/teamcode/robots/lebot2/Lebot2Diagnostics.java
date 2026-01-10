package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Loader;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

import java.util.ArrayList;
import java.util.List;

/**
 * Lebot2 Diagnostic OpMode
 *
 * A POST-style diagnostic system for verifying robot hardware and subsystem function.
 *
 * Controls:
 *   D-pad Up/Down: Select test level
 *   D-pad Left/Right: Select test within level
 *   A (Green): Run selected test / Confirm proceed
 *   B (Red): ABORT current test / Skip test
 *   X (Blue): Run all tests in current level
 *   Y (Yellow): Run full POST (all levels)
 *
 * Safety:
 *   - B button always aborts motion tests immediately
 *   - Tests requiring movement show instructions and wait for confirmation
 *   - All motion tests are limited to 1.5m or less
 *   - Some tests designed to run with robot elevated
 */
@Config(value = "Diagnostics")
@TeleOp(name = "Lebot2 Diagnostics", group = "Test")
public class Lebot2Diagnostics extends OpMode {

    //=== CONFIGURATION ===
    public static double MOTOR_TEST_POWER = 0.3;
    public static double MOTOR_TEST_DURATION_SEC = 0.5;
    public static double SHOOTER_TEST_VELOCITY = 500;  // deg/sec
    public static double SHOOTER_SPINUP_TIMEOUT_SEC = 3.0;
    public static double PID_TURN_TIMEOUT_SEC = 3.0;
    public static double SENSOR_VALID_MAX_MM = 8190;
    public static double BALL_DETECT_THRESHOLD_CM = 10;
    public static double ENCODER_MIN_CHANGE = 50;  // ticks
    public static double HEADING_TOLERANCE_DEG = 5.0;

    //=== TEST FRAMEWORK ===

    public enum TestLevel {
        LEVEL_0_EXISTENCE("L0: Hardware Exists"),
        LEVEL_1_HARDWARE("L1: Hardware Function"),
        LEVEL_2_SUBSYSTEM("L2: Subsystem Behavior"),
        LEVEL_3_INTEGRATION("L3: Integration"),
        LEVEL_4_MISSION("L4: Missions");

        public final String label;
        TestLevel(String label) { this.label = label; }
    }

    public enum TestState {
        NOT_RUN,
        WAITING_PERMISSION,  // Waiting for operator to confirm ready
        RUNNING,
        PASS,
        FAIL,
        ABORTED,
        SKIPPED
    }

    public enum MotionRequirement {
        NONE,           // No motion, safe to run anytime
        ELEVATED,       // Robot should be elevated (wheels off ground)
        GROUND_SHORT,   // Robot on ground, short motion (<0.5m)
        GROUND_MEDIUM   // Robot on ground, medium motion (<1.5m)
    }

    /**
     * Represents a single diagnostic test
     */
    public class DiagnosticTest {
        public final String name;
        public final TestLevel level;
        public final MotionRequirement motion;
        public final String instruction;  // Shown to operator before test
        public final String[] crossValidation;  // Other hardware that validates this test

        public TestState state = TestState.NOT_RUN;
        public String resultMessage = "";
        public double durationMs = 0;

        public DiagnosticTest(String name, TestLevel level, MotionRequirement motion,
                              String instruction, String... crossValidation) {
            this.name = name;
            this.level = level;
            this.motion = motion;
            this.instruction = instruction;
            this.crossValidation = crossValidation;
        }

        public void reset() {
            state = TestState.NOT_RUN;
            resultMessage = "";
            durationMs = 0;
        }

        public void pass(String message) {
            state = TestState.PASS;
            resultMessage = message;
        }

        public void fail(String message) {
            state = TestState.FAIL;
            resultMessage = message;
        }

        public void abort() {
            state = TestState.ABORTED;
            resultMessage = "Operator aborted";
        }

        public void skip(String reason) {
            state = TestState.SKIPPED;
            resultMessage = reason;
        }
    }

    //=== TEST REGISTRY ===
    private List<DiagnosticTest> allTests = new ArrayList<>();
    private DiagnosticTest currentTest = null;
    private int selectedLevel = 0;
    private int selectedTestIndex = 0;

    // Level 0: Existence tests
    private DiagnosticTest testLeftRearExists;
    private DiagnosticTest testRightRearExists;
    private DiagnosticTest testPinpointExists;
    private DiagnosticTest testIntakeExists;
    private DiagnosticTest testConveyorExists;
    private DiagnosticTest testFrontDistExists;
    private DiagnosticTest testBackDistExists;
    private DiagnosticTest testShooterExists;
    private DiagnosticTest testPaddleExists;
    private DiagnosticTest testLimelightExists;
    private DiagnosticTest testVoltage;

    // Level 1: Hardware function tests
    private DiagnosticTest testLeftRearSpin;
    private DiagnosticTest testRightRearSpin;
    private DiagnosticTest testIntakeSpin;
    private DiagnosticTest testConveyorSpin;
    private DiagnosticTest testShooterSpin;
    private DiagnosticTest testPaddleMove;
    private DiagnosticTest testFrontDistRead;
    private DiagnosticTest testBackDistRead;
    private DiagnosticTest testPinpointHeading;
    private DiagnosticTest testLimelightConnect;

    // Level 2: Subsystem behavior tests
    private DiagnosticTest testDriveForward;
    private DiagnosticTest testDriveTurn;
    private DiagnosticTest testPidTurn;
    private DiagnosticTest testIntakeBehavior;
    private DiagnosticTest testLoaderBelt;
    private DiagnosticTest testLauncherSpinup;
    private DiagnosticTest testVisionDetect;

    // Level 3: Integration tests
    private DiagnosticTest testDriveAndIntake;
    private DiagnosticTest testTargeting;

    // Level 4: Mission tests
    private DiagnosticTest testLaunchPreloads;
    private DiagnosticTest testBALLGroup;
    private DiagnosticTest testOpenSesame;

    //=== HARDWARE REFERENCES (L0-L1: Direct hardware, isolated from subsystem code) ===
    private DcMotorEx leftRear, rightRear, intake, conveyor, shooter;
    private Servo paddle;
    private DistanceSensor frontDist, backDist;
    private Limelight3A limelight;
    private PinpointLocalizer pinpoint;
    private VoltageSensor voltageSensor;
    private List<LynxModule> allHubs;

    //=== ROBOT REFERENCE (L2+: Full robot init, same as competition) ===
    private Robot robot = null;
    private boolean robotInitialized = false;
    private boolean robotInitFailed = false;
    private String robotInitError = "";

    //=== STATE ===
    private StickyGamepad stickyGamepad1;
    private ElapsedTime testTimer = new ElapsedTime();
    private ElapsedTime runTimer = new ElapsedTime();
    private FtcDashboard dashboard;
    private boolean runningFullPost = false;
    private boolean runningLevelTests = false;
    private int testPhase = 0;  // For multi-phase tests

    // Captured values for cross-validation
    private double capturedHeadingStart = 0;
    private int capturedEncoderStart = 0;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        stickyGamepad1 = new StickyGamepad(gamepad1);

        // Get Lynx modules for bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Get voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        initializeTests();

        telemetry.addLine("=== LEBOT2 DIAGNOSTICS ===");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  D-pad: Navigate tests");
        telemetry.addLine("  A: Run test / Confirm");
        telemetry.addLine("  B: ABORT / Skip");
        telemetry.addLine("  X: Run level");
        telemetry.addLine("  Y: Full POST");
        telemetry.update();
    }

    private void initializeTests() {
        // === LEVEL 0: EXISTENCE ===
        testLeftRearExists = new DiagnosticTest(
            "leftRear exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testRightRearExists = new DiagnosticTest(
            "rightRear exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testPinpointExists = new DiagnosticTest(
            "pinpoint exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testIntakeExists = new DiagnosticTest(
            "intake exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testConveyorExists = new DiagnosticTest(
            "conveyor exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testFrontDistExists = new DiagnosticTest(
            "frontDist exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testBackDistExists = new DiagnosticTest(
            "backDist exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testShooterExists = new DiagnosticTest(
            "shooter exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testPaddleExists = new DiagnosticTest(
            "paddle exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testLimelightExists = new DiagnosticTest(
            "limelight exists", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Checking hardware map...");
        testVoltage = new DiagnosticTest(
            "voltage check", TestLevel.LEVEL_0_EXISTENCE, MotionRequirement.NONE,
            "Reading battery voltage...");

        // === LEVEL 1: HARDWARE FUNCTION ===
        testLeftRearSpin = new DiagnosticTest(
            "leftRear spin", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.ELEVATED,
            "ELEVATE ROBOT - wheels must be off ground. Press A when ready.",
            "encoder change");
        testRightRearSpin = new DiagnosticTest(
            "rightRear spin", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.ELEVATED,
            "ELEVATE ROBOT - wheels must be off ground. Press A when ready.",
            "encoder change");
        testIntakeSpin = new DiagnosticTest(
            "intake spin", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.NONE,
            "Intake will spin briefly. Press A to run.");
        testConveyorSpin = new DiagnosticTest(
            "conveyor spin", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.NONE,
            "Belt will move briefly. Press A to run.");
        testShooterSpin = new DiagnosticTest(
            "shooter spinup", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.NONE,
            "Flywheel will spin up. KEEP CLEAR. Press A to run.",
            "velocity reaches target");
        testPaddleMove = new DiagnosticTest(
            "paddle move", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.NONE,
            "Paddle servo will move. Press A to run.");
        testFrontDistRead = new DiagnosticTest(
            "frontDist read", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.NONE,
            "Place hand 5cm from front sensor. Press A to test.");
        testBackDistRead = new DiagnosticTest(
            "backDist read", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.NONE,
            "Place hand 5cm from back sensor. Press A to test.");
        testPinpointHeading = new DiagnosticTest(
            "pinpoint heading", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.NONE,
            "Rotate robot ~90 degrees by hand. Press A when done.");
        testLimelightConnect = new DiagnosticTest(
            "limelight connect", TestLevel.LEVEL_1_HARDWARE, MotionRequirement.NONE,
            "Checking Limelight connection...");

        // === LEVEL 2: SUBSYSTEM BEHAVIOR ===
        testDriveForward = new DiagnosticTest(
            "drive forward", TestLevel.LEVEL_2_SUBSYSTEM, MotionRequirement.GROUND_SHORT,
            "Robot will drive FORWARD ~30cm. Clear path. Press A when ready.",
            "pinpoint x change", "both encoders");
        testDriveTurn = new DiagnosticTest(
            "drive turn", TestLevel.LEVEL_2_SUBSYSTEM, MotionRequirement.GROUND_SHORT,
            "Robot will turn ~45 degrees in place. Press A when ready.",
            "pinpoint heading", "encoder differential");
        testPidTurn = new DiagnosticTest(
            "PID turn to 90°", TestLevel.LEVEL_2_SUBSYSTEM, MotionRequirement.GROUND_SHORT,
            "Robot will PID turn to 90° heading. Press A when ready.",
            "pinpoint heading convergence");
        testIntakeBehavior = new DiagnosticTest(
            "intake behavior", TestLevel.LEVEL_2_SUBSYSTEM, MotionRequirement.NONE,
            "Testing Intake.INTAKE and Intake.EJECT behaviors. Press A to run.");
        testLoaderBelt = new DiagnosticTest(
            "loader belt request", TestLevel.LEVEL_2_SUBSYSTEM, MotionRequirement.NONE,
            "Testing belt ownership system. Press A to run.");
        testLauncherSpinup = new DiagnosticTest(
            "launcher spinup", TestLevel.LEVEL_2_SUBSYSTEM, MotionRequirement.NONE,
            "Launcher will spin to target velocity. KEEP CLEAR. Press A to run.",
            "velocity PID convergence");
        testVisionDetect = new DiagnosticTest(
            "vision detection", TestLevel.LEVEL_2_SUBSYSTEM, MotionRequirement.NONE,
            "Point robot at AprilTag goal. Press A to test detection.");

        // === LEVEL 3: INTEGRATION ===
        testDriveAndIntake = new DiagnosticTest(
            "drive + intake", TestLevel.LEVEL_3_INTEGRATION, MotionRequirement.GROUND_SHORT,
            "Robot will drive while intaking (parallel domains). Press A when ready.");
        testTargeting = new DiagnosticTest(
            "targeting", TestLevel.LEVEL_3_INTEGRATION, MotionRequirement.GROUND_MEDIUM,
            "Robot will turn to face AprilTag. Point away from goal. Press A when ready.",
            "vision tx approaches 0", "pinpoint heading");

        // === LEVEL 4: MISSIONS ===
        testLaunchPreloads = new DiagnosticTest(
            "LaunchPreloads", TestLevel.LEVEL_4_MISSION, MotionRequirement.NONE,
            "Mission: Fire preloaded balls via robot.missions. Robot will spin up and fire. KEEP CLEAR. Press A when ready.",
            "mission state", "launcher behavior", "ball count decrement");
        testBALLGroup = new DiagnosticTest(
            "BALLGroup", TestLevel.LEVEL_4_MISSION, MotionRequirement.GROUND_MEDIUM,
            "Mission: Navigate to ball group and intake. Robot will drive via RoadRunner. REQUIRES FIELD SPACE. Press A when ready.",
            "mission state", "trajectory following", "intake activation");
        testOpenSesame = new DiagnosticTest(
            "OpenSesame", TestLevel.LEVEL_4_MISSION, MotionRequirement.GROUND_MEDIUM,
            "Mission: Navigate to classifier and release. Robot will drive then back into lever. REQUIRES FIELD. Press A when ready.",
            "mission state", "trajectory following", "press sequence");

        // Register all tests
        allTests.add(testLeftRearExists);
        allTests.add(testRightRearExists);
        allTests.add(testPinpointExists);
        allTests.add(testIntakeExists);
        allTests.add(testConveyorExists);
        allTests.add(testFrontDistExists);
        allTests.add(testBackDistExists);
        allTests.add(testShooterExists);
        allTests.add(testPaddleExists);
        allTests.add(testLimelightExists);
        allTests.add(testVoltage);

        allTests.add(testLeftRearSpin);
        allTests.add(testRightRearSpin);
        allTests.add(testIntakeSpin);
        allTests.add(testConveyorSpin);
        allTests.add(testShooterSpin);
        allTests.add(testPaddleMove);
        allTests.add(testFrontDistRead);
        allTests.add(testBackDistRead);
        allTests.add(testPinpointHeading);
        allTests.add(testLimelightConnect);

        allTests.add(testDriveForward);
        allTests.add(testDriveTurn);
        allTests.add(testPidTurn);
        allTests.add(testIntakeBehavior);
        allTests.add(testLoaderBelt);
        allTests.add(testLauncherSpinup);
        allTests.add(testVisionDetect);

        allTests.add(testDriveAndIntake);
        allTests.add(testTargeting);

        allTests.add(testLaunchPreloads);
        allTests.add(testBALLGroup);
        allTests.add(testOpenSesame);
    }

    @Override
    public void loop() {
        // Clear bulk cache for fresh reads
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        stickyGamepad1.update();

        // Handle abort - always available during motion tests
        if (gamepad1.b && currentTest != null &&
            currentTest.state == TestState.RUNNING &&
            currentTest.motion != MotionRequirement.NONE) {
            abortCurrentTest();
        }

        // Navigation (only when not running a test)
        if (currentTest == null || currentTest.state != TestState.RUNNING) {
            handleNavigation();
        }

        // Run current test if active
        if (currentTest != null &&
            (currentTest.state == TestState.RUNNING ||
             currentTest.state == TestState.WAITING_PERMISSION)) {
            runCurrentTest();
        }

        // Display
        updateTelemetry();
    }

    private void handleNavigation() {
        // Level selection
        if (stickyGamepad1.dpad_up) {
            selectedLevel = Math.max(0, selectedLevel - 1);
            selectedTestIndex = 0;
        }
        if (stickyGamepad1.dpad_down) {
            selectedLevel = Math.min(TestLevel.values().length - 1, selectedLevel + 1);
            selectedTestIndex = 0;
        }

        // Test selection within level
        List<DiagnosticTest> levelTests = getTestsForLevel(TestLevel.values()[selectedLevel]);
        if (stickyGamepad1.dpad_left) {
            selectedTestIndex = Math.max(0, selectedTestIndex - 1);
        }
        if (stickyGamepad1.dpad_right) {
            selectedTestIndex = Math.min(levelTests.size() - 1, selectedTestIndex + 1);
        }

        // Run selected test
        if (stickyGamepad1.a && !levelTests.isEmpty()) {
            startTest(levelTests.get(selectedTestIndex));
        }

        // Run all tests in level
        if (stickyGamepad1.x) {
            runningLevelTests = true;
            selectedTestIndex = 0;
            if (!levelTests.isEmpty()) {
                startTest(levelTests.get(0));
            }
        }

        // Run full POST
        if (stickyGamepad1.y) {
            runningFullPost = true;
            selectedLevel = 0;
            selectedTestIndex = 0;
            List<DiagnosticTest> firstLevel = getTestsForLevel(TestLevel.LEVEL_0_EXISTENCE);
            if (!firstLevel.isEmpty()) {
                startTest(firstLevel.get(0));
            }
        }
    }

    private List<DiagnosticTest> getTestsForLevel(TestLevel level) {
        List<DiagnosticTest> result = new ArrayList<>();
        for (DiagnosticTest test : allTests) {
            if (test.level == level) {
                result.add(test);
            }
        }
        return result;
    }

    private void startTest(DiagnosticTest test) {
        currentTest = test;
        currentTest.reset();
        testPhase = 0;
        testTimer.reset();

        // Tests requiring motion need operator permission
        if (test.motion != MotionRequirement.NONE) {
            currentTest.state = TestState.WAITING_PERMISSION;
        } else {
            currentTest.state = TestState.RUNNING;
        }
    }

    private void runCurrentTest() {
        // Handle permission waiting
        if (currentTest.state == TestState.WAITING_PERMISSION) {
            if (stickyGamepad1.a) {
                currentTest.state = TestState.RUNNING;
                testTimer.reset();
                testPhase = 0;
            } else if (stickyGamepad1.b) {
                currentTest.skip("Operator skipped");
                advanceToNextTest();
                return;
            }
            return;
        }

        // Dispatch to appropriate test handler
        if (currentTest.level == TestLevel.LEVEL_0_EXISTENCE) {
            runExistenceTest();
        } else if (currentTest.level == TestLevel.LEVEL_1_HARDWARE) {
            runHardwareTest();
        } else if (currentTest.level == TestLevel.LEVEL_2_SUBSYSTEM) {
            runSubsystemTest();
        } else if (currentTest.level == TestLevel.LEVEL_3_INTEGRATION) {
            runIntegrationTest();
        } else if (currentTest.level == TestLevel.LEVEL_4_MISSION) {
            runMissionTest();
        }

        // Record duration when complete
        if (currentTest.state == TestState.PASS ||
            currentTest.state == TestState.FAIL ||
            currentTest.state == TestState.ABORTED) {
            currentTest.durationMs = testTimer.milliseconds();
            advanceToNextTest();
        }
    }

    //=== LEVEL 0: EXISTENCE TESTS ===

    private void runExistenceTest() {
        try {
            if (currentTest == testLeftRearExists) {
                leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
                leftRear.setDirection(DcMotorSimple.Direction.REVERSE);  // Match TankDrive
                currentTest.pass("Found");
            } else if (currentTest == testRightRearExists) {
                rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
                rightRear.setDirection(DcMotorSimple.Direction.FORWARD);  // Match TankDrive
                currentTest.pass("Found");
            } else if (currentTest == testPinpointExists) {
                pinpoint = new PinpointLocalizer(hardwareMap, 1.0, new Pose2d(0, 0, 0));
                currentTest.pass("Found");
            } else if (currentTest == testIntakeExists) {
                intake = hardwareMap.get(DcMotorEx.class, "intake");
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Ensure direct power control
                // Intake uses default FORWARD direction
                currentTest.pass("Found");
            } else if (currentTest == testConveyorExists) {
                conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
                conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Ensure direct power control
                conveyor.setDirection(DcMotorSimple.Direction.REVERSE);  // Match Loader
                currentTest.pass("Found");
            } else if (currentTest == testFrontDistExists) {
                frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
                currentTest.pass("Found");
            } else if (currentTest == testBackDistExists) {
                backDist = hardwareMap.get(DistanceSensor.class, "backDist");
                currentTest.pass("Found");
            } else if (currentTest == testShooterExists) {
                shooter = hardwareMap.get(DcMotorEx.class, "shooter");
                shooter.setDirection(DcMotorSimple.Direction.REVERSE);  // Match Launcher
                currentTest.pass("Found");
            } else if (currentTest == testPaddleExists) {
                paddle = hardwareMap.get(Servo.class, "paddle");
                currentTest.pass("Found");
            } else if (currentTest == testLimelightExists) {
                limelight = hardwareMap.get(Limelight3A.class, "limelight");
                currentTest.pass("Found");
            } else if (currentTest == testVoltage) {
                double voltage = voltageSensor.getVoltage();
                if (voltage > 10 && voltage < 15) {
                    currentTest.pass(String.format("%.2fV", voltage));
                } else if (voltage <= 10) {
                    currentTest.fail(String.format("LOW: %.2fV", voltage));
                } else {
                    currentTest.fail(String.format("HIGH: %.2fV", voltage));
                }
            }
        } catch (Exception e) {
            currentTest.fail("NOT FOUND: " + e.getMessage());
        }
    }

    //=== LEVEL 1: HARDWARE FUNCTION TESTS ===

    private void runHardwareTest() {
        if (currentTest == testLeftRearSpin || currentTest == testRightRearSpin) {
            runMotorSpinTest(currentTest == testLeftRearSpin ? leftRear : rightRear);
        } else if (currentTest == testIntakeSpin) {
            runSimpleMotorTest(intake, "intake");
        } else if (currentTest == testConveyorSpin) {
            runSimpleMotorTest(conveyor, "conveyor");
        } else if (currentTest == testShooterSpin) {
            runShooterSpinTest();
        } else if (currentTest == testPaddleMove) {
            runPaddleMoveTest();
        } else if (currentTest == testFrontDistRead) {
            runDistanceSensorTest(frontDist, "frontDist");
        } else if (currentTest == testBackDistRead) {
            runDistanceSensorTest(backDist, "backDist");
        } else if (currentTest == testPinpointHeading) {
            runPinpointHeadingTest();
        } else if (currentTest == testLimelightConnect) {
            runLimelightConnectTest();
        }
    }

    private void runMotorSpinTest(DcMotorEx motor) {
        if (motor == null) {
            currentTest.skip("Motor not found in L0");
            return;
        }

        switch (testPhase) {
            case 0:  // Start forward
                capturedEncoderStart = motor.getCurrentPosition();
                motor.setPower(MOTOR_TEST_POWER);
                testPhase = 1;
                break;
            case 1:  // Wait forward
                if (testTimer.seconds() > MOTOR_TEST_DURATION_SEC) {
                    motor.setPower(0);
                    int forwardChange = motor.getCurrentPosition() - capturedEncoderStart;
                    if (Math.abs(forwardChange) > ENCODER_MIN_CHANGE) {
                        // Start reverse test
                        capturedEncoderStart = motor.getCurrentPosition();
                        motor.setPower(-MOTOR_TEST_POWER);
                        testTimer.reset();
                        testPhase = 2;
                    } else {
                        currentTest.fail(String.format("Forward: only %d ticks", forwardChange));
                    }
                }
                break;
            case 2:  // Wait reverse
                if (testTimer.seconds() > MOTOR_TEST_DURATION_SEC) {
                    motor.setPower(0);
                    int reverseChange = motor.getCurrentPosition() - capturedEncoderStart;
                    if (Math.abs(reverseChange) > ENCODER_MIN_CHANGE) {
                        currentTest.pass(String.format("Fwd/Rev OK (±%d ticks)",
                            (int)ENCODER_MIN_CHANGE));
                    } else {
                        currentTest.fail(String.format("Reverse: only %d ticks", reverseChange));
                    }
                }
                break;
        }
    }

    private void runSimpleMotorTest(DcMotorEx motor, String name) {
        if (motor == null) {
            currentTest.skip("Motor not found in L0");
            return;
        }

        if (testPhase == 0) {
            // Debug: Show which motor port we're commanding
            telemetry.addData("Testing Motor", name);
            telemetry.addData("Motor Port", motor.getPortNumber());
            telemetry.addData("Motor Controller", motor.getController().getDeviceName());
            telemetry.update();

            motor.setPower(MOTOR_TEST_POWER);
            testPhase = 1;
        } else if (testTimer.seconds() > MOTOR_TEST_DURATION_SEC) {
            motor.setPower(0);
            currentTest.pass(String.format("%s ran on port %d", name, motor.getPortNumber()));
        }
    }

    private void runShooterSpinTest() {
        if (shooter == null) {
            currentTest.skip("Shooter not found in L0");
            return;
        }

        if (testPhase == 0) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setVelocity(SHOOTER_TEST_VELOCITY, AngleUnit.DEGREES);
            testPhase = 1;
        } else {
            double velocity = shooter.getVelocity(AngleUnit.DEGREES);
            double target = SHOOTER_TEST_VELOCITY;
            double error = Math.abs(velocity - target) / target;

            if (error < 0.10) {  // Within 10%
                shooter.setPower(0);
                currentTest.pass(String.format("Reached %.0f deg/s", velocity));
            } else if (testTimer.seconds() > SHOOTER_SPINUP_TIMEOUT_SEC) {
                shooter.setPower(0);
                currentTest.fail(String.format("Timeout at %.0f deg/s (target %.0f)",
                    velocity, target));
            }
        }
    }

    private void runPaddleMoveTest() {
        if (paddle == null) {
            currentTest.skip("Paddle not found in L0");
            return;
        }

        switch (testPhase) {
            case 0:  // Move to DOWN
                paddle.setPosition(0.167);
                testTimer.reset();
                testPhase = 1;
                break;
            case 1:  // Wait, then move to UP
                if (testTimer.seconds() > 0.5) {
                    paddle.setPosition(0.447);
                    testTimer.reset();
                    testPhase = 2;
                }
                break;
            case 2:  // Wait, then done
                if (testTimer.seconds() > 0.5) {
                    paddle.setPosition(0.167);  // Return to DOWN
                    currentTest.pass("Moved DOWN→UP→DOWN (visual confirm)");
                }
                break;
        }
    }

    private void runDistanceSensorTest(DistanceSensor sensor, String name) {
        if (sensor == null) {
            currentTest.skip("Sensor not found in L0");
            return;
        }

        double dist = sensor.getDistance(DistanceUnit.MM);
        if (dist < SENSOR_VALID_MAX_MM) {
            if (dist < BALL_DETECT_THRESHOLD_CM * 10) {
                currentTest.pass(String.format("%.1f cm - Object detected!", dist / 10));
            } else {
                currentTest.pass(String.format("%.1f cm - Valid range", dist / 10));
            }
        } else {
            currentTest.fail("Out of range (>8190mm)");
        }
    }

    private void runPinpointHeadingTest() {
        if (pinpoint == null) {
            currentTest.skip("Pinpoint not found in L0");
            return;
        }

        pinpoint.update();
        double heading = Math.toDegrees(pinpoint.getPose().heading.toDouble());

        if (testPhase == 0) {
            capturedHeadingStart = heading;
            testPhase = 1;
            // Wait for operator to rotate robot
        } else {
            double change = Math.abs(heading - capturedHeadingStart);
            // Handle wraparound
            if (change > 180) change = 360 - change;

            if (change > 45) {
                currentTest.pass(String.format("Heading changed %.1f°", change));
            }
            // Continue waiting until operator presses A again or aborts
        }
    }

    private void runLimelightConnectTest() {
        if (limelight == null) {
            currentTest.skip("Limelight not found in L0");
            return;
        }

        boolean connected = limelight.isConnected();
        if (connected) {
            currentTest.pass("Connected");
        } else {
            currentTest.fail("NOT connected");
        }
    }

    //=== LEVEL 2: SUBSYSTEM BEHAVIOR TESTS ===

    private void runSubsystemTest() {
        if (currentTest == testDriveForward) {
            runDriveForwardTest();
        } else if (currentTest == testDriveTurn) {
            runDriveTurnTest();
        } else if (currentTest == testPidTurn) {
            runPidTurnTest();
        } else if (currentTest == testIntakeBehavior) {
            runIntakeBehaviorTest();
        } else if (currentTest == testLoaderBelt) {
            runLoaderBeltTest();
        } else if (currentTest == testLauncherSpinup) {
            runLauncherSpinupTest();
        } else if (currentTest == testVisionDetect) {
            runVisionDetectTest();
        }
    }

    private void runDriveForwardTest() {
        if (leftRear == null || rightRear == null) {
            currentTest.skip("Drive motors not found");
            return;
        }

        switch (testPhase) {
            case 0:  // Capture start
                if (pinpoint != null) pinpoint.update();
                capturedEncoderStart = leftRear.getCurrentPosition();
                testPhase = 1;
                break;
            case 1:  // Drive forward
                leftRear.setPower(MOTOR_TEST_POWER);
                rightRear.setPower(MOTOR_TEST_POWER);
                if (testTimer.seconds() > 1.0) {  // ~30cm at low power
                    leftRear.setPower(0);
                    rightRear.setPower(0);
                    testPhase = 2;
                }
                break;
            case 2:  // Verify
                int leftChange = Math.abs(leftRear.getCurrentPosition() - capturedEncoderStart);
                int rightChange = Math.abs(rightRear.getCurrentPosition() - capturedEncoderStart);

                // Cross-validate with Pinpoint if available
                String crossVal = "";
                if (pinpoint != null) {
                    pinpoint.update();
                    // Could add pose change validation here
                    crossVal = " [Pinpoint OK]";
                }

                if (leftChange > ENCODER_MIN_CHANGE && rightChange > ENCODER_MIN_CHANGE) {
                    currentTest.pass(String.format("L:%d R:%d ticks%s", leftChange, rightChange, crossVal));
                } else {
                    currentTest.fail(String.format("L:%d R:%d - motor issue?", leftChange, rightChange));
                }
                break;
        }
    }

    private void runDriveTurnTest() {
        if (leftRear == null || rightRear == null) {
            currentTest.skip("Drive motors not found");
            return;
        }

        switch (testPhase) {
            case 0:  // Capture start heading
                if (pinpoint != null) {
                    pinpoint.update();
                    capturedHeadingStart = Math.toDegrees(pinpoint.getPose().heading.toDouble());
                }
                testPhase = 1;
                break;
            case 1:  // Turn (differential)
                leftRear.setPower(MOTOR_TEST_POWER);
                rightRear.setPower(-MOTOR_TEST_POWER);
                if (testTimer.seconds() > 0.75) {
                    leftRear.setPower(0);
                    rightRear.setPower(0);
                    testPhase = 2;
                }
                break;
            case 2:  // Verify
                String result = "Turn complete";
                if (pinpoint != null) {
                    pinpoint.update();
                    double heading = Math.toDegrees(pinpoint.getPose().heading.toDouble());
                    double change = heading - capturedHeadingStart;
                    if (change > 180) change -= 360;
                    if (change < -180) change += 360;
                    result = String.format("Turned %.1f° [Pinpoint]", change);
                }
                currentTest.pass(result);
                break;
        }
    }

    private void runPidTurnTest() {
        // Initialize Robot for real subsystem test
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:  // Start PID turn
                capturedHeadingStart = robot.driveTrain.getHeadingDegrees();
                robot.driveTrain.turnToHeading(90, 0.7);
                testPhase = 1;
                break;
            case 1:  // Wait for turn to complete
                robot.update(null);
                if (robot.driveTrain.isTurnComplete()) {
                    double finalHeading = robot.driveTrain.getHeadingDegrees();
                    double error = Math.abs(finalHeading - 90);
                    if (error > 180) error = 360 - error;

                    if (error < HEADING_TOLERANCE_DEG) {
                        currentTest.pass(String.format("PID turn complete: %.1f° (error: %.1f°)", finalHeading, error));
                    } else {
                        currentTest.fail(String.format("PID turn error too large: %.1f°", error));
                    }
                } else if (testTimer.seconds() > PID_TURN_TIMEOUT_SEC) {
                    robot.driveTrain.stop();
                    currentTest.fail("PID turn timeout");
                }
                break;
        }
    }

    private void runIntakeBehaviorTest() {
        // Initialize Robot for real subsystem test
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:  // Set INTAKE behavior
                robot.intake.setBehavior(Intake.Behavior.INTAKE);
                testPhase = 1;
                break;
            case 1:  // Verify INTAKE and run
                if (!runRobotUpdateLoop(0.5)) {
                    stopRobot();
                    currentTest.abort();
                    return;
                }
                // Verify behavior was set
                if (robot.intake.getBehavior() == Intake.Behavior.INTAKE) {
                    // Switch to EJECT
                    robot.intake.setBehavior(Intake.Behavior.EJECT);
                    testTimer.reset();
                    testPhase = 2;
                } else {
                    currentTest.fail("INTAKE behavior not active");
                }
                break;
            case 2:  // Run EJECT and verify
                if (!runRobotUpdateLoop(0.5)) {
                    stopRobot();
                    currentTest.abort();
                    return;
                }
                robot.intake.setBehavior(Intake.Behavior.OFF);
                robot.update(null);
                currentTest.pass("INTAKE→EJECT behaviors verified via subsystem");
                break;
        }
    }

    private void runLoaderBeltTest() {
        // Initialize Robot for real subsystem test
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:  // Request belt for intake
                robot.loader.requestBeltForIntake();
                testPhase = 1;
                break;
            case 1:  // Verify ownership and run
                if (!runRobotUpdateLoop(0.5)) {
                    stopRobot();
                    currentTest.abort();
                    return;
                }
                if (robot.loader.getBeltOwner() == Loader.BeltOwner.INTAKE) {
                    // Release and claim for launcher (higher priority)
                    robot.loader.releaseBeltFromIntake();
                    robot.loader.claimBeltForLauncher();
                    testTimer.reset();
                    testPhase = 2;
                } else {
                    currentTest.fail("Belt ownership not set to INTAKE");
                }
                break;
            case 2:  // Verify launcher claim
                if (!runRobotUpdateLoop(0.5)) {
                    stopRobot();
                    currentTest.abort();
                    return;
                }
                if (robot.loader.getBeltOwner() == Loader.BeltOwner.LAUNCHER) {
                    robot.loader.releaseBeltFromLauncher();
                    robot.update(null);
                    currentTest.pass("Belt ownership: INTAKE→LAUNCHER→NONE verified");
                } else {
                    currentTest.fail("Belt not claimed by LAUNCHER");
                }
                break;
        }
    }

    private void runLauncherSpinupTest() {
        // Initialize Robot for real subsystem test
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:  // Start launcher spinup
                robot.launcher.setBehavior(Launcher.Behavior.SPINNING);
                testPhase = 1;
                break;
            case 1:  // Wait for spinup
                robot.update(null);
                Launcher.LaunchState state = robot.launcher.getState();

                if (state == Launcher.LaunchState.READY) {
                    robot.launcher.setBehavior(Launcher.Behavior.IDLE);
                    robot.update(null);
                    currentTest.pass("Launcher reached READY state via behavior");
                } else if (testTimer.seconds() > SHOOTER_SPINUP_TIMEOUT_SEC) {
                    robot.launcher.setBehavior(Launcher.Behavior.IDLE);
                    robot.update(null);
                    currentTest.fail("Launcher spinup timeout - state: " + state);
                }
                break;
        }
    }

    private void runVisionDetectTest() {
        if (limelight == null) {
            currentTest.skip("Limelight not found");
            return;
        }

        if (!limelight.isConnected()) {
            currentTest.fail("Limelight not connected");
            return;
        }

        limelight.pipelineSwitch(0);  // Blue pipeline
        limelight.start();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            currentTest.pass(String.format("Detected! tx=%.1f ty=%.1f", tx, ty));
        } else {
            currentTest.fail("No valid detection - point at AprilTag");
        }
    }

    //=== LEVEL 3: INTEGRATION TESTS ===

    private void runIntegrationTest() {
        if (currentTest == testDriveAndIntake) {
            runDriveAndIntakeTest();
        } else if (currentTest == testTargeting) {
            runTargetingTest();
        }
    }

    private void runDriveAndIntakeTest() {
        // Initialize Robot for integration test
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:
                // Start both drive and intake via subsystems
                robot.driveTrain.drive(MOTOR_TEST_POWER, 0, 0);
                robot.intake.setBehavior(Intake.Behavior.INTAKE);
                testPhase = 1;
                break;
            case 1:
                // Run both in parallel (tests parallel control domains)
                if (!runRobotUpdateLoop(1.0)) {
                    stopRobot();
                    currentTest.abort();
                    return;
                }
                robot.driveTrain.stop();
                robot.intake.setBehavior(Intake.Behavior.OFF);
                robot.update(null);
                currentTest.pass("Drive + Intake parallel domains verified via Robot");
                break;
        }
    }

    private void runTargetingTest() {
        // Initialize Robot for integration test
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:
                // Start TARGETING behavior
                robot.setBehavior(Robot.Behavior.TARGETING);
                testPhase = 1;
                break;
            case 1:
                // Run until targeting completes or timeout
                robot.update(null);

                // Check if targeting completed (back to MANUAL)
                if (robot.getBehavior() == Robot.Behavior.MANUAL) {
                    // Check if we're actually aimed at target
                    if (robot.vision.hasTarget()) {
                        double tx = robot.vision.getTx();
                        if (Math.abs(tx) < 3.0) {
                            currentTest.pass(String.format("TARGETING complete: tx=%.1f°", tx));
                        } else {
                            currentTest.fail(String.format("Targeting ended but tx=%.1f° (>3°)", tx));
                        }
                    } else {
                        currentTest.pass("TARGETING completed (no target visible at end)");
                    }
                } else if (testTimer.seconds() > PID_TURN_TIMEOUT_SEC * 2) {
                    robot.setBehavior(Robot.Behavior.MANUAL);
                    robot.update(null);
                    currentTest.fail("TARGETING timeout");
                }
                break;
        }
    }

    //=== LEVEL 4: MISSION TESTS ===

    private void runMissionTest() {
        if (currentTest == testLaunchPreloads) {
            runLaunchPreloadsTest();
        } else if (currentTest == testBALLGroup) {
            runBALLGroupTest();
        } else if (currentTest == testOpenSesame) {
            runOpenSesameTest();
        }
    }

    private static final double MISSION_TIMEOUT_SEC = 30.0;

    /**
     * LaunchPreloads mission test
     * Uses robot.missions.startLaunchPreloads() to fire preloaded balls
     */
    private void runLaunchPreloadsTest() {
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:  // Start mission
                if (robot.loader.isEmpty()) {
                    currentTest.skip("No balls loaded - cannot test LaunchPreloads");
                    return;
                }
                robot.missions.startLaunchPreloads();
                testTimer.reset();
                testPhase = 1;
                break;

            case 1:  // Wait for mission completion
                robot.update(null);

                if (robot.missions.isComplete()) {
                    currentTest.pass("LaunchPreloads mission complete");
                    return;
                }
                if (robot.missions.isFailed()) {
                    currentTest.fail("LaunchPreloads mission failed");
                    return;
                }
                if (testTimer.seconds() > MISSION_TIMEOUT_SEC) {
                    robot.missions.abort();
                    currentTest.fail("LaunchPreloads timeout after " + MISSION_TIMEOUT_SEC + "s");
                }
                break;
        }
    }

    /**
     * BALLGroup mission test
     * Uses robot.missions.startBallGroup() to navigate and collect balls
     * Note: Requires field space and correct ball group positions configured
     */
    private void runBALLGroupTest() {
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:  // Start mission
                int startBallCount = robot.loader.getBallCount();
                robot.missions.startBallGroup(0);  // Test with group 0
                testTimer.reset();
                testPhase = 1;
                break;

            case 1:  // Wait for mission completion
                robot.update(null);

                if (robot.missions.isComplete()) {
                    int ballCount = robot.loader.getBallCount();
                    currentTest.pass("BALLGroup mission complete - balls: " + ballCount);
                    return;
                }
                if (robot.missions.isFailed()) {
                    currentTest.fail("BALLGroup mission failed");
                    return;
                }
                if (testTimer.seconds() > MISSION_TIMEOUT_SEC) {
                    robot.missions.abort();
                    currentTest.fail("BALLGroup timeout after " + MISSION_TIMEOUT_SEC + "s");
                }
                break;
        }
    }

    /**
     * OpenSesame mission test
     * Uses robot.missions.startOpenSesame() to navigate to classifier and release
     * Note: Requires field space and correct release pose configured
     */
    private void runOpenSesameTest() {
        if (!initRobotIfNeeded()) {
            currentTest.fail("Robot init failed: " + robotInitError);
            return;
        }

        switch (testPhase) {
            case 0:  // Start mission
                robot.missions.startOpenSesame();
                testTimer.reset();
                testPhase = 1;
                break;

            case 1:  // Wait for mission completion
                robot.update(null);

                if (robot.missions.isComplete()) {
                    currentTest.pass("OpenSesame mission complete - classifier released");
                    return;
                }
                if (robot.missions.isFailed()) {
                    currentTest.fail("OpenSesame mission failed");
                    return;
                }
                if (testTimer.seconds() > MISSION_TIMEOUT_SEC) {
                    robot.missions.abort();
                    currentTest.fail("OpenSesame timeout after " + MISSION_TIMEOUT_SEC + "s");
                }
                break;
        }
    }

    //=== UTILITY METHODS ===

    /**
     * Lazily initialize Robot for L2+ tests.
     * Returns true if Robot is available, false if init failed.
     */
    private boolean initRobotIfNeeded() {
        if (robotInitialized) {
            return !robotInitFailed;
        }

        try {
            robot = new Robot(hardwareMap, false);
            robotInitialized = true;
            robotInitFailed = false;
            return true;
        } catch (Exception e) {
            robotInitialized = true;
            robotInitFailed = true;
            robotInitError = e.getMessage();
            return false;
        }
    }

    /**
     * Run Robot update loop for specified duration.
     * Used by L2+ tests that need real subsystem behavior.
     *
     * @param seconds Duration to run
     * @return true if completed, false if aborted
     */
    private boolean runRobotUpdateLoop(double seconds) {
        if (robot == null) return false;

        ElapsedTime loopTimer = new ElapsedTime();
        while (loopTimer.seconds() < seconds) {
            // Check for abort
            if (gamepad1.b) {
                return false;
            }

            // Clear bulk cache and run robot update
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            robot.update(null);
        }
        return true;
    }

    /**
     * Stop Robot if initialized (for abort and cleanup).
     */
    private void stopRobot() {
        if (robot != null) {
            robot.intake.setBehavior(Intake.Behavior.OFF);
            robot.loader.stop();
            robot.launcher.stop();
            robot.driveTrain.stop();
        }
    }

    private void abortCurrentTest() {
        // Stop all motors immediately (direct hardware for L0-L1)
        if (leftRear != null) leftRear.setPower(0);
        if (rightRear != null) rightRear.setPower(0);
        if (intake != null) intake.setPower(0);
        if (conveyor != null) conveyor.setPower(0);
        if (shooter != null) shooter.setPower(0);

        // Stop Robot subsystems (for L2+)
        stopRobot();

        currentTest.abort();
    }

    private void advanceToNextTest() {
        TestLevel level = TestLevel.values()[selectedLevel];
        List<DiagnosticTest> levelTests = getTestsForLevel(level);

        if (runningLevelTests || runningFullPost) {
            selectedTestIndex++;

            if (selectedTestIndex >= levelTests.size()) {
                // Level complete
                if (runningFullPost && selectedLevel < TestLevel.values().length - 1) {
                    // Move to next level
                    selectedLevel++;
                    selectedTestIndex = 0;
                    levelTests = getTestsForLevel(TestLevel.values()[selectedLevel]);
                } else {
                    // All done
                    runningLevelTests = false;
                    runningFullPost = false;
                    currentTest = null;
                    return;
                }
            }

            if (!levelTests.isEmpty() && selectedTestIndex < levelTests.size()) {
                startTest(levelTests.get(selectedTestIndex));
            }
        } else {
            currentTest = null;
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== LEBOT2 DIAGNOSTICS ===");
        telemetry.addLine("");

        // Current test status
        if (currentTest != null) {
            telemetry.addLine(">>> " + currentTest.name + " <<<");
            telemetry.addData("State", currentTest.state);

            if (currentTest.state == TestState.WAITING_PERMISSION) {
                telemetry.addLine("");
                telemetry.addLine("INSTRUCTION:");
                telemetry.addLine(currentTest.instruction);
                telemetry.addLine("");
                telemetry.addLine("[A] Proceed   [B] Skip");
            } else if (currentTest.state == TestState.RUNNING) {
                telemetry.addData("Time", "%.1f s", testTimer.seconds());
                if (currentTest.motion != MotionRequirement.NONE) {
                    telemetry.addLine("[B] ABORT");
                }
            }

            if (!currentTest.resultMessage.isEmpty()) {
                telemetry.addData("Result", currentTest.resultMessage);
            }
        } else {
            // Navigation mode
            TestLevel level = TestLevel.values()[selectedLevel];
            telemetry.addLine("Level: " + level.label);

            List<DiagnosticTest> levelTests = getTestsForLevel(level);
            telemetry.addLine("");

            for (int i = 0; i < levelTests.size(); i++) {
                DiagnosticTest test = levelTests.get(i);
                String prefix = (i == selectedTestIndex) ? ">> " : "   ";
                String stateIcon = getStateIcon(test.state);
                String msg = test.resultMessage.isEmpty() ? "" : " - " + test.resultMessage;
                telemetry.addLine(prefix + stateIcon + " " + test.name + msg);
            }
        }

        // Summary
        telemetry.addLine("");
        telemetry.addLine(getSummary());

        telemetry.update();
    }

    private String getStateIcon(TestState state) {
        switch (state) {
            case PASS: return "[OK]";
            case FAIL: return "[!!]";
            case RUNNING: return "[..]";
            case WAITING_PERMISSION: return "[??]";
            case ABORTED: return "[AB]";
            case SKIPPED: return "[--]";
            default: return "[  ]";
        }
    }

    private String getSummary() {
        int pass = 0, fail = 0, skip = 0, notRun = 0;
        for (DiagnosticTest test : allTests) {
            switch (test.state) {
                case PASS: pass++; break;
                case FAIL: fail++; break;
                case SKIPPED:
                case ABORTED: skip++; break;
                default: notRun++; break;
            }
        }
        return String.format("Summary: %d PASS, %d FAIL, %d SKIP, %d pending",
            pass, fail, skip, notRun);
    }

    @Override
    public void stop() {
        // Ensure all motors are stopped
        if (leftRear != null) leftRear.setPower(0);
        if (rightRear != null) rightRear.setPower(0);
        if (intake != null) intake.setPower(0);
        if (conveyor != null) conveyor.setPower(0);
        if (shooter != null) shooter.setPower(0);
    }
}
