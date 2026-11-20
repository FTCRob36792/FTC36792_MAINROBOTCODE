package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * BIOBUZZ Combined TeleOp (BASE driving + MainDev shooter/feeder/vision features)
 *
 * PS5 DualSense / FTC Gamepad 1 controls (standard FTC button fields):
 *  - Left stick: field-centric mecanum forward/back and strafe
 *  - Right stick X: robot rotation
 *  - Cross (A): combined AprilTag approach + yaw alignment; press again to cancel
 *  - Circle (B): webcam yellow-ball approach; press again to cancel
 *  - Square (X) during INIT: toggle RED / BLUE alliance
 *  - Square (X) during TeleOp: toggle feeder servo between positions 1 and 2
 *  - Triangle (Y) / Options: re-zero field heading
 *  - D-pad Left: toggle field-centric / robot-centric driving
 *  - R1 + D-pad: individual wheel test
 *  - L1: intake OUT; L2: intake IN
 *  - R2: shooter ON (with optional distance-based power)
 *  - D-pad Up / Down (without R1): feeder servo position 1 / position 2
 *
 * Retains BASE's mecanum drive, Pinpoint field-centric behavior, ball detection/approach,
 * and AprilTag positional approach. Adds MainDev's shooter, feeder servo, battery
 * compensation, advanced Limelight filtering, distance-based shooter power, and
 * velocity-lead yaw correction.
 */
@TeleOp(name = "BIOBUZZ ROBERTA", group = "BIOBUZZ")
public class roberta extends LinearOpMode {

    @com.acmerobotics.dashboard.config.Config
    public static class Config {
        // =====================================================================
        // 1. SHOOTER & FEEDER SUBSYSTEM (MOST FREQUENTLY TUNED)
        // =====================================================================

        // ---------- Shooter Motor Speeds & Mid-Shot Power Bump ----------
        public static boolean SHOOTER_ENABLED = true;          // Enable or disable shooter motor control loop
        public static boolean REVERSE_SHOOTER = false;         // Reverse shooter motor spinning direction
        public static double SHOOTER_BASE_POWER = 0.85;        // Base shooter motor power before mid-shot bump (85%)
        public static double SHOOTER_BUMP_POWER = 0.95;        // Bumped shooter motor power mid-shot (95%)
        public static boolean SHOOTER_BUMP_ENABLED = false;     // Enable shooter power bump mid-shot
        public static long SHOOTER_BUMP_DELAY_MS = 150;        // Timed delay in ms after servo actuates (Pos 2) before bumping power
        public static long SHOOTER_BUMP_DURATION_MS = 500;     // Duration in ms to hold power bump (0 = hold while servo is extended)
        public static double SHOOTER_TARGET_POWER = 0.85;      // Target motor power fallback / legacy alias
        public static long SHOOTER_RAMP_UP_MS = 0;             // Power ramp-up duration in milliseconds (0 = instant speed)

        // ---------- Auto Shooter Distance-Speed Calibration ----------
        public static boolean SHOOTER_AUTO_SPEED_ENABLED = false; // Auto-adjust shooter power based on Limelight target distance
        public static double SHOOTER_CLOSE_DIST_IN = 36.0;        // Close calibration distance in inches (e.g. 3 feet)
        public static double SHOOTER_CLOSE_POWER = 0.60;          // Shooter motor power at close distance (e.g. 60%)
        public static double SHOOTER_FAR_DIST_IN = 120.0;         // Far calibration distance in inches (e.g. 10 feet)
        public static double SHOOTER_FAR_POWER = 1.00;            // Shooter motor power at far distance (e.g. 100%)
        public static double SHOOTER_MIN_POWER = 0.40;            // Minimum clamped shooter power limit
        public static double SHOOTER_MAX_POWER = 1.00;            // Maximum clamped shooter power limit

        // ---------- Shooter Battery Voltage Compensation ----------
        public static boolean SHOOTER_VOLTAGE_COMP_ENABLED = false; // Automatic voltage compensation to maintain constant RPM under voltage sags
        public static double NOMINAL_BATTERY_VOLTAGE = 12.0;       // Target nominal battery voltage (volts)

        // ---------- Shooter Feeder Servo Control & Ramping ----------
        public static boolean SERVO_ENABLED = true;            // Enable or disable custom servo control loop
        public static boolean REVERSE_SERVO = false;           // Reverse physical servo rotation direction
        public static double SERVO_ANGLE_POS1_DEG = 0.0;      // Target Position 1 angle in degrees (default rest / retracted position)
        public static double SERVO_ANGLE_POS2_DEG = 90.0;     // Target Position 2 angle in degrees (default active / extended position)
        public static double SERVO_ANGLE_MIN_DEG = 0.0;       // Physical minimum mechanical angle of servo in degrees (maps to 0.0 position)
        public static double SERVO_ANGLE_MAX_DEG = 180.0;     // Physical maximum mechanical angle of servo in degrees (maps to 1.0 position)
        public static double SERVO_SPEED_DEG_PER_SEC = 0.0;   // Maximum speed limit in degrees per second (0 = instant hardware max speed)
        public static double SERVO_RAMP_ACCEL_DEG_SEC2 = 0.0;  // Acceleration limit in deg/sec² for smooth speed ramping (0 = instant speed change)
        public static double SERVO_RAMP_DECEL_DISTANCE_DEG = 0.0; // Distance in degrees to begin decelerating smoothly near target angle (0 = no decel ramp)
        public static boolean SERVO_AUTO_TIMER_ENABLED = true;// True = automatically return servo to Position 1 after holding Position 2
        public static long SERVO_TIMER_HOLD_MS = 5000;        // Hold duration in milliseconds at Position 2 before auto-returning to Position 1 (0 = disabled)

        // =====================================================================
        // 2. INTAKE SUBSYSTEM
        // =====================================================================
        public static boolean REVERSE_INTAKE = false;         // Reverse intake motor spinning direction
        public static double INTAKE_IN_POWER = 0.8;           // Motor power when pulling items IN (Left Trigger)
        public static double INTAKE_OUT_POWER = 0.9;          // Motor power when ejecting items OUT (Left Bumper)
        public static double TRIGGER_THRESHOLD = 0.1;         // Trigger threshold to activate intake / shooter

        // =====================================================================
        // 3. VISION & AUTO-AIM SUBSYSTEM (LIMELIGHT 3A & APRILTAGS)
        // =====================================================================
        public static boolean ENABLE_APRILTAG_AUTO_ALIGN = false;// Master lock for Gamepad 1 [A] AprilTag auto-align feature
        public static int APRILTAG_PIPELINE = 0;              // Limelight pipeline index configured for AprilTag detection
        public static String RED_ALIGN_TAG_IDS = "31,35";     // Comma-separated list of allowed AprilTag IDs on RED alliance
        public static String BLUE_ALIGN_TAG_IDS = "39,43";    // Comma-separated list of allowed AprilTag IDs on BLUE alliance
        public static double ALIGN_YAW_KP = 0.015;            // Proportional turn power gain per degree of horizontal yaw offset (tx)
        public static double ALIGN_YAW_TOLERANCE_DEG = 1.0;   // Horizontal yaw alignment complete tolerance in degrees
        public static double ALIGN_MAX_TURN = 0.35;           // Maximum turn power limit during yaw alignment
        public static long ALIGN_TIMEOUT_MS = 6000;           // Maximum alignment duration in milliseconds

        // ---------- BASE positional approach target / motion control ----------
        public static double ALIGN_TARGET_FORWARD_IN = 50.0;  // Desired tag position ahead of robot center
        public static double ALIGN_TARGET_RIGHT_IN = 40.5;    // Desired tag position right of robot center
        public static boolean BLUE_MIRRORS_SIDE = true;       // Mirror target-right offset for BLUE alliance
        public static double ALIGN_TOLERANCE_IN = 1.5;        // Positional completion tolerance
        public static double ALIGN_KP = 0.04;                 // Translation speed per inch of position error
        public static double ALIGN_MAX_SPEED = 0.50;
        public static double ALIGN_MIN_SPEED = 0.15;
        public static boolean ALIGN_HOLD_HEADING = false;
        public static double ALIGN_HEADING_KP = 0.01;
        public static double ALIGN_MAX_HEADING_TURN = 0.15;   // Separate cap for Pinpoint heading hold
        public static double ALIGN_BLIND_MAX_IN = 30.0;       // Maximum remaining distance allowed for Pinpoint fallback
        public static double ALIGN_DIVERGE_IN = 8.0;          // Cancel if alignment error increases this much

        // ---------- Velocity Lead / Follow Aiming (Shooting on the Move) ----------
        public static boolean LEAD_AIM_ENABLED = false;          // Master toggle to enable/disable velocity lead/follow aiming compensation
        public static double LEAD_AIM_GAIN_DEG_PER_IPS = 0.5;   // Lead angle gain in degrees per inch/sec of robot strafe velocity (positive = lead, negative = follow)
        public static double LEAD_AIM_MAX_OFFSET_DEG = 15.0;    // Hard maximum limit on lead/follow angle offset in degrees
        public static double LEAD_AIM_VELOCITY_DEADZONE_IPS = 1.0; // Minimum robot velocity threshold in inches/sec required to activate lead compensation
        public static boolean LEAD_AIM_USE_STICK_ESTIMATE = false;// Fallback to joystick input velocity estimate if Pinpoint odometry is disconnected
        public static double LEAD_AIM_STICK_MAX_IPS = 40.0;     // Estimated max robot strafe velocity in inches/sec when using joystick fallback

        // ---------- Advanced Limelight 3D Calibration ----------
        public static boolean IGNORE_TIPPED_HIVES = true;      // Ignore AprilTags on tipped hives (tilted roll/pitch orientation)
        public static double TIPPED_HIVE_MAX_ROLL_DEG = 45.0;  // Maximum roll/pitch tilt in degrees before tag is ignored as a tipped hive
        public static boolean USE_ROBOT_SPACE_POSE = true;    // True = getTargetPoseRobotSpace() [X=fwd, Y=right, Z=up], False = getTargetPoseCameraSpace() [X=right, Y=down, Z=fwd]
        public static int TAG_FORWARD_AXIS = 0;               // Axis index for forward motion (0 = X, 1 = Y, 2 = Z)
        public static int TAG_RIGHT_AXIS = 1;                 // Axis index for sideways right motion (0 = X, 1 = Y, 2 = Z)
        public static boolean TAG_FORWARD_NEGATE = false;     // Negate forward target position if axis direction is inverted
        public static boolean TAG_RIGHT_NEGATE = false;       // Negate right target position if axis direction is inverted

        // =====================================================================
        // 4. VISION & BALL APPROACH SUBSYSTEM (LOGITECH WEBCAM)
        // =====================================================================
        public static boolean ENABLE_BLOB_AUTO_APPROACH = false;// Master lock for Gamepad 1 [B] Yellow ball approach feature
        public static boolean SHOW_BALL_MASK = false;         // True = display binary vision mask on camera preview for tuning
        public static int HSV_H_MIN = 20;                     // Minimum Hue threshold (0-180) for yellow color detection
        public static int HSV_H_MAX = 38;                     // Maximum Hue threshold (0-180) for yellow color detection
        public static int HSV_S_MIN = 80;                     // Minimum Saturation threshold (0-255) for yellow color detection
        public static int HSV_S_MAX = 255;                    // Maximum Saturation threshold (0-255) for yellow color detection
        public static int HSV_V_MIN = 60;                     // Minimum Value/Brightness threshold (0-255) for yellow color detection
        public static int HSV_V_MAX = 255;                    // Maximum Value/Brightness threshold (0-255) for yellow color detection
        public static double MIN_BALL_AREA_PX = 40.0;         // Minimum area in pixels for a valid yellow blob
        public static double CLUSTER_DIST_DIAMETERS = 5.0;    // Maximum cluster distance in ball diameters to group nearby blobs
        public static double WEBCAM_HFOV_DEG = 60.0;          // Horizontal field-of-view of webcam in degrees

        // ---------- Ball Auto-Approach Controls ----------
        public static boolean BALL_CAMERA_ON_BACK = false;    // False = camera is mounted on front, True = camera mounted on back
        public static boolean REVERSE_BALL_STRAFE = false;    // Reverse sideways strafe direction during ball approach
        public static double BALL_STOP_Y_FRACTION = 0.97;     // Stop threshold when ball cluster bottom reaches image height fraction (0 = top, 1 = bottom)
        public static double BALL_FORWARD_KP = 4.0;           // Proportional gain for forward drive power per image height error
        public static double BALL_STRAFE_KP = 0.04;           // Proportional gain for strafe power per degree off-center
        public static double BALL_CENTER_TOLERANCE_DEG = 3.0; // Centering tolerance in degrees
        public static double BALL_MAX_SPEED = 1.00;           // Maximum forward drive speed toward balls
        public static double MIN_BALL_FORWARD = 0.40;         // Minimum forward drive power toward balls
        public static double BALL_MAX_STRAFE = 0.80;          // Maximum strafe power toward balls
        public static double BALL_MIN_STRAFE = 0.30;          // Minimum strafe power toward balls
        public static boolean BALL_HOLD_HEADING = true;       // True = lock heading with Pinpoint while approaching balls
        public static double BALL_HEADING_KP = 0.01;          // Proportional gain for heading hold during ball approach
        public static double BALL_MAX_TURN = 0.15;            // Maximum turn power limit during ball approach heading hold
        public static int BALL_CAPACITY = 4;                  // Maximum ball storage capacity of robot
        public static long BALL_BLIND_PUSH_MS = 300;          // Final blind push duration in milliseconds when balls slip below camera view
        public static double BALL_BLIND_MIN_Y = 0.85;         // Minimum Y image fraction required to trigger final blind push
        public static long BALL_TIMEOUT_MS = 6000;            // Ball approach timeout in milliseconds

        // ---------- Webcam Camera Settings ----------
        public static boolean WEBCAM_MANUAL_EXPOSURE = false;  // True = enable manual camera exposure/gain controls, False = auto exposure
        public static long WEBCAM_EXPOSURE_MS = 15;            // Camera exposure time in milliseconds (when manual exposure is enabled)
        public static int WEBCAM_GAIN = 100;                   // Camera gain level (when manual exposure is enabled)

        // =====================================================================
        // 5. DRIVETRAIN & FIELD-CENTRIC DRIVING
        // =====================================================================
        public static boolean FIELD_CENTRIC = true;          // True = swerve style (stick direction relative to field), False = robot-centric
        public static double STICK_DEADZONE = 0.1;             // Joystick deadzone threshold (stick inputs below this absolute value are ignored)
        public static boolean REVERSE_HEADING = false;       // Flip heading direction if field drive moves wrong way after turning
        public static boolean REVERSE_STRAFE_STICK = false;  // Flip left stick X direction if strafing is backwards
        public static boolean INVERT_FORWARD = false;          // True = opposite end of robot is front (flips forward & strafe commands together)
        public static boolean REVERSE_DRIVE_FORWARD = false;   // Flip forward / backward driving commands
        public static boolean REVERSE_DRIVE_TURN = false;      // Flip clockwise / counter-clockwise turning commands
        public static boolean REVERSE_FRONT_LEFT = true;       // Flip rotation direction of Front Left drive motor
        public static boolean REVERSE_FRONT_RIGHT = false;     // Flip rotation direction of Front Right drive motor
        public static boolean REVERSE_BACK_LEFT = true;        // Flip rotation direction of Back Left drive motor
        public static boolean REVERSE_BACK_RIGHT = false;      // Flip rotation direction of Back Right drive motor

        // =====================================================================
        // 6. MATCH SETUP & INITIAL ROBOT POSE
        // =====================================================================
        public static boolean RED_ALLIANCE = true;             // Default alliance selection (True = RED, False = BLUE)
        public static double START_X_IN = 0.0;                 // Initial X coordinate in inches for Pinpoint pose reset
        public static double START_Y_IN = 0.0;                 // Initial Y coordinate in inches for Pinpoint pose reset
        public static double START_HEADING_DEG = 0.0;          // Initial heading in degrees for Pinpoint pose reset

        // =====================================================================
        // 7. PINPOINT ODOMETRY & MISC HARDWARE
        // =====================================================================
        public static double PINPOINT_X_POD_OFFSET_MM = 0.0; // X pod offset from robot center in millimeters
        public static double PINPOINT_Y_POD_OFFSET_MM = 0.0; // Y pod offset from robot center in millimeters
        public static boolean REVERSE_X_ENCODER = false;       // Reverse X pod encoder counting direction
        public static boolean REVERSE_Y_ENCODER = false;       // Reverse Y pod encoder counting direction
        // Off 0.0 | Red 0.277 | Orange 0.333 | Yellow 0.388 | Sage 0.444 | Green 0.500 | Azure 0.555 | Blue 0.611 | Indigo 0.666 | Violet 0.722 | White 1.0
        public static double RGB_LIGHT_POSITION = 0.555;      // Preset servo position for electric blue indicator color (#4BB4CB)
        public static long ALLIANCE_SHOW_DURATION_MS = 1500;   // Hold duration in ms to display selected alliance color (Red/Blue) when changed in init
        public static long INIT_BLINK_INTERVAL_MS = 1000;       // Quick blink interval in ms for blue light during init
        public static long TELEOP_BLINK_INTERVAL_MS = 250;     // Blink interval in ms for blue light during normal TeleOp
        public static double ALIGN_LIGHT_MAX_ERROR_DEG = 15.0; // Yaw error in degrees corresponding to Red (0% aligned)
        public static double END_GAME_PARK_WARNING_SEC = 5.0;  // Seconds before end of match (120s total) to trigger park warning light cycle
        public static long PARK_WARNING_STROBE_MS = 70;        // Strobe color cycle step interval in ms for park alert

        // =====================================================================
        // 8. HARDWARE CONFIGURATION NAMES (ROBOT CONFIG)
        // =====================================================================
        public static String FRONT_LEFT_NAME = "front_left";   // Hardware configuration name for Front Left drive motor
        public static String FRONT_RIGHT_NAME = "back_right";  // Hardware configuration name for Front Right drive motor
        public static String BACK_LEFT_NAME = "back_left";     // Hardware configuration name for Back Left drive motor
        public static String BACK_RIGHT_NAME = "front_right";  // Hardware configuration name for Front Right drive motor
        public static String PINPOINT_NAME = "pinpoint";       // Hardware configuration name for goBILDA Pinpoint Odometry Computer
        public static String LIMELIGHT_NAME = "limelight";     // Hardware configuration name for Limelight 3A vision sensor
        public static String WEBCAM_NAME = "Webcam 1";         // Hardware configuration name for Logitech/OpenCV USB Webcam
        public static String RGB_LIGHT_NAME = "rgb_light";     // Hardware configuration name for goBILDA RGB Indicator Light (Servo)
        public static String INTAKE_NAME = "intake";           // Hardware configuration name for Intake motor
        public static String SHOOTER_NAME = "shooter";         // Hardware configuration name for Shooter motor
        public static String SERVO_NAME = "servo";             // Hardware configuration name for tuneable custom Servo
    }

    // ---------- Hardware Devices ----------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private DcMotor shooter;
    private Servo customServo;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private OpenCvWebcam webcam;
    private VoltageSensor batteryVoltageSensor;
    private Servo rgbLight;
    private double lastRgbPosition = -1.0;

    // ---------- Motor & Actuator Caching for Bus & Power Efficiency ----------
    private double lastCustomServoPos = -999;
    private double lastIntakePower = -999;

    // ---------- Tuneable Servo State & Speed Ramping ----------
    private double targetServoAngleDeg = 0.0;
    private double currentServoAngleDeg = 0.0;
    private double currentServoVelocityDegPerSec = 0.0;
    private long lastServoUpdateMs = 0;
    private boolean servoStatePos1 = true;

    // ---------- Tuneable Servo Timer State ----------
    private long servoPos2StartMs = 0;
    private boolean servoTimerActive = false;

    // ---------- Tuneable Shooter Ramping & Distance State ----------
    private long shooterRampStartMs = 0;
    private double currentShooterPower = 0.0;
    private double latestTargetDistanceIn = 0.0;

    // ---------- Battery Voltage Throttling ----------
    private double cachedBatteryVoltage = 12.0;
    private long lastVoltageReadMs = 0;

    private double getBatteryVoltage(long now) {
        if (batteryVoltageSensor != null && (now - lastVoltageReadMs > 250 || lastVoltageReadMs == 0)) {
            try {
                double v = batteryVoltageSensor.getVoltage();
                if (v > 1.0) {
                    cachedBatteryVoltage = v;
                }
            } catch (Exception ignored) {
            }
            lastVoltageReadMs = now;
        }
        return cachedBatteryVoltage;
    }

    // ---------- Webcam Ball Detection ----------
    private final BallPipeline ballPipeline = new BallPipeline();
    private volatile BallTarget latestBalls;
    private volatile long latestBallsMs = 0;
    private volatile String webcamStatus = "NOT INITIALIZED";

    // ---------- OpMode State Variables ----------
    private boolean autoAlignActive = false;
    private boolean blobApproachActive = false;
    private boolean previousA = false, previousB = false, previousY = false, previousX = false;
    private boolean previousOptions = false, previousInitX = false, previousDpadLeft = false;

    private double fieldHeadingOffsetDeg = 0.0;
    private double lastXOffset = Double.NaN, lastYOffset = Double.NaN;
    private boolean lastXEncoderRev = false, lastYEncoderRev = false;
    private int lastLimelightPipeline = -1;

    private boolean lastManualExposure = false;
    private long lastExposureMs = -1;
    private int lastGain = -1;

    private String assistStatus = "READY";
    private long statusHoldUntilMs = 0;
    private String tagDebug = "";
    private boolean alignTagSeen = false;
    private long alignStartMs = 0;
    private double alignHoldHeadingDeg = 0.0;
    private double alignLastErrF = 0.0, alignLastErrR = 0.0;
    private double alignLastSeenX = 0.0, alignLastSeenY = 0.0;
    private double alignMinErr = Double.MAX_VALUE;
    private double alignStartErr = -1.0;
    private long lastAlignTagSeenMs = 0;
    private boolean ballSeen = false;
    private double ballHoldHeadingDeg = 0.0;
    private double lastBallY = 0.0;
    private long ballBlindStartMs = 0;
    private long ballStartMs = 0;
    private String alignDebug = "";

    private boolean allianceLocked = false;
    private boolean lockedRed = true;
    private boolean lastInitRedAlliance = true;
    private long allianceShowStartMs = 0;

    private String activeTagIds() {
        boolean red = allianceLocked ? lockedRed : Config.RED_ALLIANCE;
        return red ? Config.RED_ALIGN_TAG_IDS : Config.BLUE_ALIGN_TAG_IDS;
    }

    private double targetRightIn() {
        return (Config.BLUE_MIRRORS_SIDE && !lockedRed)
                ? -Config.ALIGN_TARGET_RIGHT_IN : Config.ALIGN_TARGET_RIGHT_IN;
    }
    private long lastBlobSeenMs = 0;

    /** Robot heading in degrees (counter-clockwise positive), flipped if REVERSE_HEADING is on. */
    private double headingDeg() {
        if (pinpoint == null) return 0.0;
        double h = pinpoint.getHeading(AngleUnit.DEGREES);
        return Config.REVERSE_HEADING ? -h : h;
    }

    @Override
    public void runOpMode() {
        Config.FIELD_CENTRIC = true; // Dashboard values stay until the app restarts, so always start in field-centric
        initializeHardware();
        configurePinpoint();

        lastInitRedAlliance = Config.RED_ALLIANCE;
        allianceShowStartMs = System.currentTimeMillis();

        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(Config.APRILTAG_PIPELINE);
            limelight.start();
            lastLimelightPipeline = Config.APRILTAG_PIPELINE;
        }
        initializeWebcam();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        fieldHeadingOffsetDeg = Config.START_HEADING_DEG;

        // ---------- INIT LOOP ----------
        while (!isStarted() && !isStopRequested()) {
            long now = System.currentTimeMillis();
            if (gamepad1.x && !previousInitX) {
                Config.RED_ALLIANCE = !Config.RED_ALLIANCE;
            }
            previousInitX = gamepad1.x;

            if (gamepad1.dpad_left && !previousDpadLeft) {
                Config.FIELD_CENTRIC = !Config.FIELD_CENTRIC;
            }
            previousDpadLeft = gamepad1.dpad_left;

            if (Config.RED_ALLIANCE != lastInitRedAlliance) {
                lastInitRedAlliance = Config.RED_ALLIANCE;
                allianceShowStartMs = now;
            }

            updatePinpointConfigIfNeeded();
            updateLimelightConfigIfNeeded();
            updateWebcamExposureIfNeeded();
            updateInitIndicatorLight(now);
            if (pinpoint != null) {
                pinpoint.update();
            }

            updateInitTelemetry();
            sleep(20);
        }

        // Lock the field-centric zero once, at the end of INIT.
        // Do not continuously chase the sensor heading during INIT.
        if (pinpoint != null) {
            pinpoint.update();
            fieldHeadingOffsetDeg = headingDeg();
        } else {
            fieldHeadingOffsetDeg = Config.START_HEADING_DEG;
        }

        lockedRed = Config.RED_ALLIANCE;
        allianceLocked = true;

        long teleOpStartMs = System.currentTimeMillis();

        // ---------- MAIN LOOP ----------
        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            updatePinpointConfigIfNeeded();
            updateLimelightConfigIfNeeded();
            updateWebcamExposureIfNeeded();
            updateCustomServo(now);

            double robotX = 0.0, robotY = 0.0, robotHeadingDeg = 0.0;
            if (pinpoint != null) {
                pinpoint.update();
                robotX = pinpoint.getPosX(DistanceUnit.INCH);
                robotY = pinpoint.getPosY(DistanceUnit.INCH);
                robotHeadingDeg = headingDeg();
            }

            handleButtons(now);

            runIntake();
            runShooter(now);

            boolean manualOverride = hasManualStickInput();
            if (manualOverride && (autoAlignActive || blobApproachActive)) {
                cancelAssist("MANUAL OVERRIDE");
            }

            TagView tagView = readAprilTags();
            BallTarget ballTarget = (latestBalls != null && now - latestBallsMs <= 700)
                    ? latestBalls : null;

            double matchTimeRemainingSec = 120.0 - ((now - teleOpStartMs) / 1000.0);
            updateTeleOpIndicatorLight(now, matchTimeRemainingSec, tagView);

            if (autoAlignActive) {
                runAprilTagAlign(tagView, now, robotX, robotY, robotHeadingDeg);
            } else if (blobApproachActive) {
                runBallApproach(ballTarget, now, robotHeadingDeg);
            } else {
                boolean wheelTest = runFieldCentricDrive(robotHeadingDeg);
                if (!wheelTest && !assistStatus.equals("HEADING RE-ZEROED") && now >= statusHoldUntilMs) {
                    assistStatus = manualOverride ? "MANUAL DRIVE" : "READY";
                }
            }

            updateTelemetry(robotX, robotY, robotHeadingDeg, tagView, ballTarget);
            idle();
        }

        // ---------- SHUTDOWN ----------
        stopDrive();
        if (intake != null) intake.setPower(0.0);
        if (shooter != null) shooter.setPower(0.0);
        if (limelight != null) limelight.stop();
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    // =====================================================================
    // SETUP
    // =====================================================================

    // goBILDA RGB indicator servo position presets
    private static final double COLOR_OFF_POS = 0.0;
    private static final double COLOR_RED_POS = 0.277;
    private static final double COLOR_ORANGE_POS = 0.333;
    private static final double COLOR_YELLOW_POS = 0.388;
    private static final double COLOR_GREEN_POS = 0.500;
    private static final double COLOR_BLUE_POS = 0.611;

    // Eyecatching multi-color cycle for end-of-match park warning
    private static final double[] PARK_WARNING_COLORS = {
            0.277,  // Red
            0.333,  // Orange
            0.388,  // Yellow
            0.500,  // Green
            0.555,  // Azure/Blue
            0.722,  // Violet
            1.000   // White
    };

    private void setIndicatorLightPosition(double position) {
        if (rgbLight == null) return;

        double safePosition = clip(position, 0.0, 1.0);
        if (Math.abs(safePosition - lastRgbPosition) >= 0.0001) {
            rgbLight.setPosition(safePosition);
            lastRgbPosition = safePosition;
        }
    }

    /**
     * Updates indicator light during INIT phase:
     * - Displays selected alliance color (Solid Red or Solid Blue) for a short period (1500ms)
     *   whenever alliance selection is initialized or toggled.
     * - Otherwise, displays a quick blinking blue color.
     */
    private void updateInitIndicatorLight(long now) {
        if (rgbLight == null) return;

        if (now - allianceShowStartMs < Config.ALLIANCE_SHOW_DURATION_MS) {
            // Display alliance color for visual confirmation to drive team
            double allianceColor = Config.RED_ALLIANCE ? COLOR_RED_POS : COLOR_BLUE_POS;
            setIndicatorLightPosition(allianceColor);
        } else {
            // Quick blinking solid blue
            boolean blinkOn = ((now / Config.INIT_BLINK_INTERVAL_MS) % 2) == 0;
            double blueColor = Config.RGB_LIGHT_POSITION > 0 ? Config.RGB_LIGHT_POSITION : COLOR_BLUE_POS;
            setIndicatorLightPosition(blinkOn ? blueColor : COLOR_OFF_POS);
        }
    }

    /**
     * Updates indicator light during TeleOp phase:
     * 1. At set time before end of match (5 seconds): eyecatching rapid color cycle to alert driver to park.
     * 2. Auto-align active: smooth transition from Red (0% aligned) through Orange/Yellow to Green (100% aligned).
     * 3. Otherwise: blinks solid blue color.
     */
    private void updateTeleOpIndicatorLight(long now, double matchTimeRemainingSec, TagView tagView) {
        if (rgbLight == null) return;

        // 1. End of match park alert (final N seconds of match)
        if (matchTimeRemainingSec <= Config.END_GAME_PARK_WARNING_SEC && matchTimeRemainingSec >= -10.0) {
            int colorIndex = (int) ((now / Config.PARK_WARNING_STROBE_MS) % PARK_WARNING_COLORS.length);
            setIndicatorLightPosition(PARK_WARNING_COLORS[colorIndex]);
            return;
        }

        // 2. Auto-align to hive color transition (Red -> Orange -> Yellow -> Green)
        if (autoAlignActive) {
            double progress = 0.0;
            if (tagView != null && alignTagSeen) {
                // Determine current yaw alignment error (including velocity lead aim offset)
                double strafeVelIPS = 0.0;
                if (pinpoint != null) {
                    strafeVelIPS = pinpoint.getVelY(DistanceUnit.INCH);
                } else if (Config.LEAD_AIM_USE_STICK_ESTIMATE) {
                    double rawStickX = Config.REVERSE_STRAFE_STICK ? -gamepad1.left_stick_x : gamepad1.left_stick_x;
                    strafeVelIPS = rawStickX * Config.LEAD_AIM_STICK_MAX_IPS;
                }
                double leadOffsetDeg = 0.0;
                if (Config.LEAD_AIM_ENABLED && Math.abs(strafeVelIPS) >= Config.LEAD_AIM_VELOCITY_DEADZONE_IPS) {
                    leadOffsetDeg = clip(strafeVelIPS * Config.LEAD_AIM_GAIN_DEG_PER_IPS,
                            -Config.LEAD_AIM_MAX_OFFSET_DEG, Config.LEAD_AIM_MAX_OFFSET_DEG);
                }
                double effectiveYawErrDeg = Math.abs(tagView.yawErrorDeg - leadOffsetDeg);

                double maxErr = Math.max(Config.ALIGN_YAW_TOLERANCE_DEG + 0.1, Config.ALIGN_LIGHT_MAX_ERROR_DEG);
                double minErr = Config.ALIGN_YAW_TOLERANCE_DEG;

                // 0.0 when error >= maxErr (Red), 1.0 when error <= minErr (Green)
                progress = 1.0 - clip((effectiveYawErrDeg - minErr) / (maxErr - minErr), 0.0, 1.0);
            } else {
                // Auto-align active but tag not seen yet -> Red (0% progress)
                progress = 0.0;
            }

            // Smooth interpolation from Red (0.277) to Green (0.500)
            double alignColorPos = COLOR_RED_POS + progress * (COLOR_GREEN_POS - COLOR_RED_POS);
            setIndicatorLightPosition(alignColorPos);
            return;
        }

        // 3. Normal driving mode: blinking blue
        boolean blinkOn = ((now / Config.TELEOP_BLINK_INTERVAL_MS) % 2) == 0;
        double blueColor = Config.RGB_LIGHT_POSITION > 0 ? Config.RGB_LIGHT_POSITION : COLOR_BLUE_POS;
        setIndicatorLightPosition(blinkOn ? blueColor : COLOR_OFF_POS);
    }

    private DcMotor getDriveMotor(String name) {
        try {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return motor;
        } catch (Exception e) {
            return null;
        }
    }

    private void initializeHardware() {
        frontLeft = getDriveMotor(Config.FRONT_LEFT_NAME);
        frontRight = getDriveMotor(Config.FRONT_RIGHT_NAME);
        backLeft = getDriveMotor(Config.BACK_LEFT_NAME);
        backRight = getDriveMotor(Config.BACK_RIGHT_NAME);

        try {
            rgbLight = hardwareMap.get(Servo.class, Config.RGB_LIGHT_NAME);
        } catch (Exception e) {
            rgbLight = null;
        }

        try {
            customServo = hardwareMap.get(Servo.class, Config.SERVO_NAME);
            targetServoAngleDeg = Config.SERVO_ANGLE_POS1_DEG;
            currentServoAngleDeg = Config.SERVO_ANGLE_POS1_DEG;
            currentServoVelocityDegPerSec = 0.0;
        } catch (Exception e) {
            customServo = null;
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Config.PINPOINT_NAME);
        } catch (Exception e) {
            pinpoint = null;
        }

        try {
            limelight = hardwareMap.get(Limelight3A.class, Config.LIMELIGHT_NAME);
        } catch (Exception e) {
            limelight = null;
        }

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            batteryVoltageSensor = sensor;
            break;
        }

        try {
            intake = hardwareMap.get(DcMotor.class, Config.INTAKE_NAME);
            intake.setDirection(Config.REVERSE_INTAKE ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            intake = null;
        }

        try {
            shooter = hardwareMap.get(DcMotor.class, Config.SHOOTER_NAME);
            shooter.setDirection(Config.REVERSE_SHOOTER ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            shooter = null;
        }
    }

    private void initializeWebcam() {
        try {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, Config.WEBCAM_NAME);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            webcam.setPipeline(ballPipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    FtcDashboard.getInstance().startCameraStream(webcam, 10);
                    webcamStatus = "STREAMING 320x240";
                }

                @Override
                public void onError(int errorCode) {
                    webcamStatus = "CAMERA ERROR " + errorCode;
                }
            });
        } catch (Exception e) {
            webcam = null;
            webcamStatus = "NOT FOUND";
        }
    }

    private void configurePinpoint() {
        if (pinpoint == null) return;

        applyPinpointSettings();

        // Set the configured starting pose first.
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Config.START_X_IN, Config.START_Y_IN,
                AngleUnit.DEGREES, Config.START_HEADING_DEG));

        // A bad Pinpoint gyro zero-offset can make heading drift while stationary.
        // The robot MUST remain completely still during this calibration.
        pinpoint.recalibrateIMU();
        sleep(400); // goBILDA specifies approximately 250 ms; allow a little margin.
        pinpoint.update();
    }

    private void applyPinpointSettings() {
        pinpoint.setOffsets(Config.PINPOINT_X_POD_OFFSET_MM, Config.PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                Config.REVERSE_X_ENCODER ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Config.REVERSE_Y_ENCODER ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);

        lastXOffset = Config.PINPOINT_X_POD_OFFSET_MM;
        lastYOffset = Config.PINPOINT_Y_POD_OFFSET_MM;
        lastXEncoderRev = Config.REVERSE_X_ENCODER;
        lastYEncoderRev = Config.REVERSE_Y_ENCODER;
    }

    private void updatePinpointConfigIfNeeded() {
        if (pinpoint == null) return;
        if (Config.PINPOINT_X_POD_OFFSET_MM != lastXOffset || Config.PINPOINT_Y_POD_OFFSET_MM != lastYOffset
                || Config.REVERSE_X_ENCODER != lastXEncoderRev || Config.REVERSE_Y_ENCODER != lastYEncoderRev) {
            applyPinpointSettings();
        }
    }

    private void updateLimelightConfigIfNeeded() {
        if (limelight == null) return;
        if (Config.APRILTAG_PIPELINE != lastLimelightPipeline) {
            limelight.pipelineSwitch(Config.APRILTAG_PIPELINE);
            lastLimelightPipeline = Config.APRILTAG_PIPELINE;
        }
    }

    private void updateWebcamExposureIfNeeded() {
        if (webcam == null) return;
        try {
            ExposureControl exposureControl = webcam.getExposureControl();
            GainControl gainControl = webcam.getGainControl();

            if (exposureControl != null && (Config.WEBCAM_MANUAL_EXPOSURE != lastManualExposure
                    || Config.WEBCAM_EXPOSURE_MS != lastExposureMs || Config.WEBCAM_GAIN != lastGain)) {

                if (Config.WEBCAM_MANUAL_EXPOSURE) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    exposureControl.setExposure(Config.WEBCAM_EXPOSURE_MS, TimeUnit.MILLISECONDS);
                    if (gainControl != null) {
                        gainControl.setGain(Math.min(gainControl.getMaxGain(),
                                Math.max(gainControl.getMinGain(), Config.WEBCAM_GAIN)));
                    }
                } else {
                    exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
                }
                lastManualExposure = Config.WEBCAM_MANUAL_EXPOSURE;
                lastExposureMs = Config.WEBCAM_EXPOSURE_MS;
                lastGain = Config.WEBCAM_GAIN;
            }
        } catch (Exception ignored) {
        }
    }

    /**
     * Updates tuneable servo position with smooth speed limiting, acceleration/deceleration ramping,
     * and tunable auto-return timer.
     */
    private void updateCustomServo(long now) {
        if (customServo == null || !Config.SERVO_ENABLED) return;

        if (lastServoUpdateMs == 0) {
            lastServoUpdateMs = now;
            return;
        }

        double dtSec = (now - lastServoUpdateMs) / 1000.0;
        lastServoUpdateMs = now;

        if (dtSec <= 0.0) return;

        // Auto-return timer check when servo is holding Position 2
        if (Config.SERVO_AUTO_TIMER_ENABLED && servoTimerActive && !servoStatePos1 && Config.SERVO_TIMER_HOLD_MS > 0) {
            if (now - servoPos2StartMs >= Config.SERVO_TIMER_HOLD_MS) {
                servoStatePos1 = true;
                servoTimerActive = false;
                servoPos2StartMs = 0;
            }
        }

        targetServoAngleDeg = servoStatePos1 ? Config.SERVO_ANGLE_POS1_DEG : Config.SERVO_ANGLE_POS2_DEG;

        double distanceRemaining = Math.abs(targetServoAngleDeg - currentServoAngleDeg);
        double dir = Math.signum(targetServoAngleDeg - currentServoAngleDeg);

        if (distanceRemaining < 0.01) {
            currentServoAngleDeg = targetServoAngleDeg;
            currentServoVelocityDegPerSec = 0.0;
        } else if (Config.SERVO_SPEED_DEG_PER_SEC <= 0.0 || Config.SERVO_SPEED_DEG_PER_SEC >= 720.0) {
            // Instant hardware max speed
            currentServoAngleDeg = targetServoAngleDeg;
            currentServoVelocityDegPerSec = 0.0;
        } else {
            // Calculate top allowable speed for this frame based on deceleration distance ramp
            double maxSpeed = Config.SERVO_SPEED_DEG_PER_SEC;

            // Deceleration ramp: smooth slowdown as servo approaches target angle
            if (Config.SERVO_RAMP_DECEL_DISTANCE_DEG > 0.0 && distanceRemaining < Config.SERVO_RAMP_DECEL_DISTANCE_DEG) {
                double decelRamp = distanceRemaining / Config.SERVO_RAMP_DECEL_DISTANCE_DEG;
                maxSpeed = Math.max(10.0, maxSpeed * decelRamp);
            }

            // Acceleration ramp: limit velocity change by acceleration limit
            if (Config.SERVO_RAMP_ACCEL_DEG_SEC2 > 0.0) {
                double maxVelChange = Config.SERVO_RAMP_ACCEL_DEG_SEC2 * dtSec;
                if (currentServoVelocityDegPerSec < maxSpeed) {
                    currentServoVelocityDegPerSec = Math.min(maxSpeed, currentServoVelocityDegPerSec + maxVelChange);
                } else {
                    currentServoVelocityDegPerSec = Math.max(maxSpeed, currentServoVelocityDegPerSec - maxVelChange);
                }
            } else {
                currentServoVelocityDegPerSec = maxSpeed;
            }

            // Step angle by current velocity
            double angleStep = currentServoVelocityDegPerSec * dtSec;
            if (angleStep >= distanceRemaining) {
                currentServoAngleDeg = targetServoAngleDeg;
                currentServoVelocityDegPerSec = 0.0;
            } else {
                currentServoAngleDeg += dir * angleStep;
            }
        }

        double range = Math.max(0.001, Config.SERVO_ANGLE_MAX_DEG - Config.SERVO_ANGLE_MIN_DEG);
        double normPos = (currentServoAngleDeg - Config.SERVO_ANGLE_MIN_DEG) / range;
        normPos = clip(normPos, 0.0, 1.0);
        if (Config.REVERSE_SERVO) {
            normPos = 1.0 - normPos;
        }

        // Caching: Only call hardware setPosition if position changed to save bus traffic & power
        if (Math.abs(normPos - lastCustomServoPos) >= 0.0005) {
            customServo.setPosition(normPos);
            lastCustomServoPos = normPos;
        }
    }

    // =====================================================================
    // INPUT
    // =====================================================================

    private void handleButtons(long now) {
        boolean pressedA = gamepad1.a && !previousA;
        boolean pressedB = gamepad1.b && !previousB;
        boolean pressedY = gamepad1.y && !previousY;
        boolean pressedX = gamepad1.x && !previousX;
        boolean optionsHeld = gamepad1.options || gamepad1.back;
        boolean pressedOptions = optionsHeld && !previousOptions;

        if (pressedA) {
            if (autoAlignActive) {
                cancelAssist("ALIGN CANCELLED");
            } else if (!Config.ENABLE_APRILTAG_AUTO_ALIGN) {
                assistStatus = "ALIGN DISABLED (Config)";
            } else if (limelight == null) {
                assistStatus = "NO LIMELIGHT";
            } else {
                blobApproachActive = false;
                autoAlignActive = true;
                alignTagSeen = false;
                alignStartMs = now;
                alignHoldHeadingDeg = headingDeg();
                alignMinErr = Double.MAX_VALUE;
                alignStartErr = -1.0;
                alignLastErrF = 0.0;
                alignLastErrR = 0.0;
                lastAlignTagSeenMs = now;
                assistStatus = "LOOKING FOR CELL TAGS (POSITION + YAW)";
            }
        }

        if (pressedB) {
            if (blobApproachActive) {
                cancelAssist("BALL APPROACH CANCELLED");
            } else if (!Config.ENABLE_BLOB_AUTO_APPROACH) {
                assistStatus = "BALL APPROACH DISABLED (Config)";
            } else {
                autoAlignActive = false;
                blobApproachActive = true;
                ballSeen = false;
                lastBallY = 0.0;
                ballBlindStartMs = 0;
                ballHoldHeadingDeg = headingDeg();
                ballStartMs = now;
                lastBlobSeenMs = now;
                assistStatus = "LOOKING FOR BALLS";
            }
        }

        if (pressedX) {
            servoStatePos1 = !servoStatePos1;
            if (!servoStatePos1) {
                servoPos2StartMs = now;
                servoTimerActive = true;
            } else {
                servoTimerActive = false;
                servoPos2StartMs = 0;
            }
        }

        // D-Pad Up / Down (when Right Bumper is not held for motor test) toggles custom servo position
        if (!gamepad1.right_bumper) {
            if (gamepad1.dpad_up) {
                servoStatePos1 = true;
                servoTimerActive = false;
                servoPos2StartMs = 0;
            } else if (gamepad1.dpad_down) {
                servoStatePos1 = false;
                servoPos2StartMs = now;
                servoTimerActive = true;
            }
        }

        if (pressedY || pressedOptions) {
            if (pinpoint != null) fieldHeadingOffsetDeg = headingDeg();
            assistStatus = "HEADING RE-ZEROED";
            statusHoldUntilMs = now + 1500;
        } else if (!gamepad1.y && !optionsHeld && assistStatus.equals("HEADING RE-ZEROED") && now >= statusHoldUntilMs) {
            assistStatus = "READY";
        }

        boolean pressedDpadLeft = gamepad1.dpad_left && !previousDpadLeft;
        if (!gamepad1.right_bumper && pressedDpadLeft) {
            Config.FIELD_CENTRIC = !Config.FIELD_CENTRIC;
            assistStatus = Config.FIELD_CENTRIC ? "FIELD-CENTRIC ON" : "ROBOT-CENTRIC ON";
        }
        previousDpadLeft = gamepad1.dpad_left;

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousY = gamepad1.y;
        previousX = gamepad1.x;
        previousOptions = optionsHeld;
    }

    private boolean hasManualStickInput() {
        return Math.abs(gamepad1.left_stick_x) > Config.STICK_DEADZONE
                || Math.abs(gamepad1.left_stick_y) > Config.STICK_DEADZONE
                || Math.abs(gamepad1.right_stick_x) > Config.STICK_DEADZONE;
    }

    private void cancelAssist(String reason) {
        autoAlignActive = false;
        blobApproachActive = false;
        assistStatus = reason;
        statusHoldUntilMs = System.currentTimeMillis() + 4000;
        stopDrive();
    }

    // =====================================================================
    // DRIVE
    // =====================================================================

    /** Returns true if the wheel test is running this loop. */
    private boolean runFieldCentricDrive(double robotHeadingDeg) {
        // Wheel test: hold RB + a D-Pad direction
        if (gamepad1.right_bumper) {
            double power = 0.35;
            double fl = gamepad1.dpad_up ? power : 0.0;
            double fr = gamepad1.dpad_right ? power : 0.0;
            double bl = gamepad1.dpad_down ? power : 0.0;
            double br = gamepad1.dpad_left ? power : 0.0;

            if (fl != 0 || fr != 0 || bl != 0 || br != 0) {
                setMotorPowersDirect(fl, fr, bl, br);
                if (gamepad1.dpad_up) assistStatus = "MOTOR TEST: FRONT LEFT";
                else if (gamepad1.dpad_right) assistStatus = "MOTOR TEST: FRONT RIGHT";
                else if (gamepad1.dpad_down) assistStatus = "MOTOR TEST: BACK LEFT";
                else assistStatus = "MOTOR TEST: BACK RIGHT";
                return true;
            }
        }

        // Left stick = move (up/down/left/right), right stick X = turn
        double stickForward = Math.abs(gamepad1.left_stick_y) > Config.STICK_DEADZONE ? -gamepad1.left_stick_y : 0.0;
        double stickRight = Math.abs(gamepad1.left_stick_x) > Config.STICK_DEADZONE ? gamepad1.left_stick_x : 0.0;
        if (Config.REVERSE_STRAFE_STICK) stickRight = -stickRight;
        double turn = Math.abs(gamepad1.right_stick_x) > Config.STICK_DEADZONE ? gamepad1.right_stick_x : 0.0;

        if (stickForward == 0.0 && stickRight == 0.0 && turn == 0.0) {
            stopDrive(); // zero power + BRAKE mode = motors resist spinning
            return false;
        }

        double forward = stickForward;
        double strafeRight = stickRight;

        // Swerve style: stick directions are relative to the field/driver, not the robot
        if (Config.FIELD_CENTRIC && pinpoint != null) {
            double rel = Math.toRadians(normalizeDegrees(robotHeadingDeg - fieldHeadingOffsetDeg));
            strafeRight = stickRight * Math.cos(rel) + stickForward * Math.sin(rel);
            forward = -stickRight * Math.sin(rel) + stickForward * Math.cos(rel);
        }

        setRobotDrive(forward, strafeRight, turn);
        return false;
    }

    private void setRobotDrive(double forward, double strafeRight, double turn) {
        // INVERT_FORWARD is the 180-degree robot-front option; it flips both translation axes.
        if (Config.INVERT_FORWARD) {
            forward = -forward;
            strafeRight = -strafeRight;
        }

        // These independent corrections match the observed drive behavior without changing strafing.
        // Keeping them here applies the same directions to manual drive and both assist routines.
        if (Config.REVERSE_DRIVE_FORWARD) forward = -forward;
        if (Config.REVERSE_DRIVE_TURN) turn = -turn;

        double fl = forward + strafeRight + turn;
        double fr = forward - strafeRight - turn;
        double bl = forward - strafeRight + turn;
        double br = forward + strafeRight - turn;
        setMotorPowersDirect(fl, fr, bl, br);
    }

    private void setMotorPowersDirect(double fl, double fr, double bl, double br) {
        if (Config.REVERSE_FRONT_LEFT) fl = -fl;
        if (Config.REVERSE_FRONT_RIGHT) fr = -fr;
        if (Config.REVERSE_BACK_LEFT) bl = -bl;
        if (Config.REVERSE_BACK_RIGHT) br = -br;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        if (frontLeft != null) frontLeft.setPower(fl / max);
        if (frontRight != null) frontRight.setPower(fr / max);
        if (backLeft != null) backLeft.setPower(bl / max);
        if (backRight != null) backRight.setPower(br / max);
    }

    private void runIntake() {
        if (intake == null) return;
        double targetPower = 0.0;
        if (gamepad1.left_bumper) {
            targetPower = -Config.INTAKE_OUT_POWER;
        } else if (gamepad1.left_trigger > Config.TRIGGER_THRESHOLD) {
            targetPower = Config.INTAKE_IN_POWER;
        }
        if (Math.abs(targetPower - lastIntakePower) >= 0.001) {
            intake.setPower(targetPower);
            lastIntakePower = targetPower;
        }
    }

    private void runShooter(long now) {
        if (shooter == null) return;
        double targetPower = 0.0;
        if (Config.SHOOTER_ENABLED && gamepad1.right_trigger > Config.TRIGGER_THRESHOLD) {
            if (shooterRampStartMs == 0) {
                shooterRampStartMs = now;
            }

            // Start with base speed (85% or distance-based auto speed)
            double basePower = Config.SHOOTER_BASE_POWER;
            if (Config.SHOOTER_AUTO_SPEED_ENABLED && latestTargetDistanceIn > 0.0) {
                double range = Math.max(0.1, Math.abs(Config.SHOOTER_FAR_DIST_IN - Config.SHOOTER_CLOSE_DIST_IN));
                double t = (latestTargetDistanceIn - Config.SHOOTER_CLOSE_DIST_IN) / range;
                double autoPower = Config.SHOOTER_CLOSE_POWER + t * (Config.SHOOTER_FAR_POWER - Config.SHOOTER_CLOSE_POWER);
                basePower = clip(autoPower, Config.SHOOTER_MIN_POWER, Config.SHOOTER_MAX_POWER);
            }

            double fullTargetPower = basePower;

            // Mid-shot power bump logic: check if servo is actuated (Pos 2) and bump delay has elapsed
            if (Config.SHOOTER_BUMP_ENABLED && !servoStatePos1 && servoPos2StartMs > 0) {
                long elapsedSinceActuation = now - servoPos2StartMs;
                if (elapsedSinceActuation >= Config.SHOOTER_BUMP_DELAY_MS &&
                        (Config.SHOOTER_BUMP_DURATION_MS <= 0 || elapsedSinceActuation < (Config.SHOOTER_BUMP_DELAY_MS + Config.SHOOTER_BUMP_DURATION_MS))) {
                    if (Config.SHOOTER_AUTO_SPEED_ENABLED && latestTargetDistanceIn > 0.0) {
                        double bumpDelta = Config.SHOOTER_BUMP_POWER - Config.SHOOTER_BASE_POWER;
                        fullTargetPower = clip(basePower + bumpDelta, Config.SHOOTER_MIN_POWER, Config.SHOOTER_MAX_POWER);
                    } else {
                        fullTargetPower = Config.SHOOTER_BUMP_POWER;
                    }
                }
            }

            // Optional battery voltage compensation to maintain target output as battery sags under load
            if (Config.SHOOTER_VOLTAGE_COMP_ENABLED) {
                double volts = getBatteryVoltage(now);
                if (volts > 8.0) {
                    double compFactor = Config.NOMINAL_BATTERY_VOLTAGE / volts;
                    fullTargetPower = clip(fullTargetPower * compFactor, 0.0, Config.SHOOTER_MAX_POWER);
                }
            }

            double rampFrac = Config.SHOOTER_RAMP_UP_MS > 0
                    ? clip((double) (now - shooterRampStartMs) / Config.SHOOTER_RAMP_UP_MS, 0.0, 1.0)
                    : 1.0;
            targetPower = fullTargetPower * rampFrac;
        } else {
            shooterRampStartMs = 0;
        }

        if (Math.abs(targetPower - currentShooterPower) >= 0.001 || (targetPower == 0.0 && currentShooterPower != 0.0)) {
            currentShooterPower = targetPower;
            shooter.setPower(currentShooterPower);
        }
    }

    private void stopDrive() {
        setMotorPowersDirect(0.0, 0.0, 0.0, 0.0);
    }

    // =====================================================================
    // APRILTAG ALIGN & DISTANCE TARGETING (BIOBUZZ CELLS)
    // =====================================================================

    private static class TagView {
        final int tagId;
        final double forwardIn;   // + = tag is toward robot front
        final double rightIn;     // + = tag is to robot right
        final double yawErrorDeg; // Horizontal target angle tx in degrees
        final double distanceIn;  // Distance in inches from robot to tag
        final double rawX, rawY, rawZ;
        final int tagCount;

        TagView(int tagId, double forwardIn, double rightIn, double yawErrorDeg, double distanceIn,
                double rawX, double rawY, double rawZ, int tagCount) {
            this.tagId = tagId;
            this.forwardIn = forwardIn;
            this.rightIn = rightIn;
            this.yawErrorDeg = yawErrorDeg;
            this.distanceIn = distanceIn;
            this.rawX = rawX;
            this.rawY = rawY;
            this.rawZ = rawZ;
            this.tagCount = tagCount;
        }
    }

    private int[] allowedTagIds = new int[0];
    private String allowedTagIdsSource = null;

    private boolean isAllowedTag(int id) {
        String ids = activeTagIds();
        if (!ids.equals(allowedTagIdsSource)) {
            allowedTagIdsSource = ids;
            String[] parts = ids.split(",");
            int[] parsed = new int[parts.length];
            int n = 0;
            for (String part : parts) {
                try {
                    parsed[n] = Integer.parseInt(part.trim());
                    n++;
                } catch (NumberFormatException ignored) {
                }
            }
            allowedTagIds = java.util.Arrays.copyOf(parsed, n);
        }
        for (int allowed : allowedTagIds) {
            if (allowed == id) return true;
        }
        return false;
    }

    /**
     * Checks if a tag belongs to a tipped hive (tilted roll/pitch orientation).
     */
    private static boolean isTippedHive(LLResultTypes.FiducialResult tag) {
        if (!Config.IGNORE_TIPPED_HIVES) return false;
        try {
            Pose3D pose = Config.USE_ROBOT_SPACE_POSE ? tag.getTargetPoseRobotSpace() : tag.getTargetPoseCameraSpace();
            if (pose == null || pose.getOrientation() == null) return false;
            double roll = pose.getOrientation().getRoll(AngleUnit.DEGREES);
            double pitch = pose.getOrientation().getPitch(AngleUnit.DEGREES);
            return Math.abs(normalizeDegrees(roll)) > Config.TIPPED_HIVE_MAX_ROLL_DEG
                    || Math.abs(normalizeDegrees(pitch)) > Config.TIPPED_HIVE_MAX_ROLL_DEG;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Extracts 3D target pose in inches.
     * Returns null if pose is null or if Limelight returned an uninitialized/dummy (0,0,0) pose.
     */
    private static double[] tagPoseInches(LLResultTypes.FiducialResult tag) {
        try {
            Pose3D pose = Config.USE_ROBOT_SPACE_POSE ? tag.getTargetPoseRobotSpace() : tag.getTargetPoseCameraSpace();
            if (pose == null || pose.getPosition() == null) return null;
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);

            // Filter out uninitialized / dummy zero pose returned when 3D solvePnP fails
            if (Math.abs(p.x) < 1e-4 && Math.abs(p.y) < 1e-4 && Math.abs(p.z) < 1e-4) {
                return null;
            }
            return new double[]{p.x, p.y, p.z};
        } catch (Exception e) {
            return null;
        }
    }

    private TagView readAprilTags() {
        if (limelight == null) {
            tagDebug = "NO LIMELIGHT IN CONFIG";
            latestTargetDistanceIn = 0.0;
            return null;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            tagDebug = "NO RESULT";
            latestTargetDistanceIn = 0.0;
            return null;
        }
        if (result.getPipelineIndex() != Config.APRILTAG_PIPELINE) {
            tagDebug = "WRONG PIPELINE: Limelight is on " + result.getPipelineIndex()
                    + ", Config wants " + Config.APRILTAG_PIPELINE;
            latestTargetDistanceIn = 0.0;
            return null;
        }
        if (!result.isValid()) {
            tagDebug = "NO TARGET SEEN";
            latestTargetDistanceIn = 0.0;
            return null;
        }
        if (limelight.getTimeSinceLastUpdate() > 500) {
            tagDebug = "STALE DATA (" + limelight.getTimeSinceLastUpdate() + "ms)";
            latestTargetDistanceIn = 0.0;
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            tagDebug = "NO TAGS IN RESULT";
            latestTargetDistanceIn = 0.0;
            return null;
        }

        int forwardAxis = Math.max(0, Math.min(2, Config.TAG_FORWARD_AXIS));
        int rightAxis = Math.max(0, Math.min(2, Config.TAG_RIGHT_AXIS));
        StringBuilder seen = new StringBuilder();
        int allowedCount = 0;
        TagView best = null;
        double bestPositionError = Double.MAX_VALUE;

        for (LLResultTypes.FiducialResult tag : fiducials) {
            seen.append(tag.getFiducialId()).append(' ');
            if (!isAllowedTag(tag.getFiducialId())) continue;
            if (isTippedHive(tag)) continue;
            allowedCount++;

            double[] raw = tagPoseInches(tag);
            if (raw == null) continue;

            double forward = raw[forwardAxis] * (Config.TAG_FORWARD_NEGATE ? -1.0 : 1.0);
            double right = raw[rightAxis] * (Config.TAG_RIGHT_NEGATE ? -1.0 : 1.0);
            double distanceIn = Math.hypot(forward, right);
            double yawErrorDeg = tag.getTargetXDegrees();

            // Prefer the tag whose position is closest to the BASE alignment destination.
            double positionError = Math.hypot(
                    forward - Config.ALIGN_TARGET_FORWARD_IN,
                    right - targetRightIn());
            if (positionError < bestPositionError) {
                bestPositionError = positionError;
                best = new TagView(tag.getFiducialId(), forward, right, yawErrorDeg,
                        distanceIn, raw[0], raw[1], raw[2], allowedCount);
            }
        }

        if (best == null) {
            tagDebug = allowedCount == 0
                    ? "SEES TAG IDs [" + seen.toString().trim() + "] BUT NONE VALID IN ALIGN_TAG_IDS (" + activeTagIds() + ")"
                    : "TAG SEEN BUT TIPPED OR NO 3D POSE";
            latestTargetDistanceIn = 0.0;
            return null;
        }

        latestTargetDistanceIn = best.distanceIn;
        tagDebug = String.format(Locale.US, "OK - Tag #%d (dist %.1f in, tx %+.1f°)",
                best.tagId, best.distanceIn, best.yawErrorDeg);
        return new TagView(best.tagId, best.forwardIn, best.rightIn, best.yawErrorDeg,
                best.distanceIn, best.rawX, best.rawY, best.rawZ, allowedCount);
    }

    /**
     * Combined BASE positional AprilTag approach and MainDev yaw/velocity-lead alignment.
     * The robot drives toward the configured tag-relative destination while correcting yaw.
     * If the tag is briefly lost near the destination, BASE's Pinpoint blind-finish behavior is retained.
     */
    private void runAprilTagAlign(TagView tag, long now,
                                  double robotX, double robotY, double robotHeadingDeg) {
        if (now - alignStartMs > Config.ALIGN_TIMEOUT_MS) {
            cancelAssist("ALIGN TIMEOUT");
            return;
        }

        double tolerance = Math.max(0.5, Config.ALIGN_TOLERANCE_IN);
        double errForward;
        double errRight;
        boolean tagInView = tag != null;

        if (tagInView) {
            alignTagSeen = true;
            lastAlignTagSeenMs = now;
            errForward = tag.forwardIn - Config.ALIGN_TARGET_FORWARD_IN;
            errRight = tag.rightIn - targetRightIn();
            alignLastErrForward(errForward, errRight, robotX, robotY);
        } else {
            if (!alignTagSeen) {
                stopDrive();
                assistStatus = "NO TAG YET - POINT LIMELIGHT AT AN UNTIPPED ALLIANCE TAG";
                if (now - alignStartMs > 3000) cancelAssist("NO UNTIPPED ALLIANCE TAG FOUND");
                return;
            }

            double lastDistance = Math.hypot(alignLastErrF, alignLastErrR);
            if (pinpoint == null || lastDistance > Config.ALIGN_BLIND_MAX_IN) {
                cancelAssist("TAG LOST");
                return;
            }
            if (lastDistance < 1e-6) {
                cancelAssist("ALIGNED (tag lost)");
                return;
            }

            double traveled = Math.hypot(robotX - alignLastSeenX, robotY - alignLastSeenY);
            double remaining = lastDistance - traveled;
            if (remaining <= tolerance) {
                cancelAssist("ALIGNED (tag out of view)");
                return;
            }
            errForward = alignLastErrF * (remaining / lastDistance);
            errRight = alignLastErrR * (remaining / lastDistance);
        }

        double distance = Math.hypot(errForward, errRight);

        // MainDev velocity-lead calculation. Pinpoint is preferred; stick estimate is the fallback.
        double strafeVelIPS = 0.0;
        if (pinpoint != null) {
            strafeVelIPS = pinpoint.getVelY(DistanceUnit.INCH);
        } else if (Config.LEAD_AIM_USE_STICK_ESTIMATE) {
            double rawStickX = Config.REVERSE_STRAFE_STICK
                    ? -gamepad1.left_stick_x : gamepad1.left_stick_x;
            strafeVelIPS = rawStickX * Config.LEAD_AIM_STICK_MAX_IPS;
        }

        double leadOffsetDeg = 0.0;
        if (Config.LEAD_AIM_ENABLED
                && Math.abs(strafeVelIPS) >= Config.LEAD_AIM_VELOCITY_DEADZONE_IPS) {
            leadOffsetDeg = clip(strafeVelIPS * Config.LEAD_AIM_GAIN_DEG_PER_IPS,
                    -Config.LEAD_AIM_MAX_OFFSET_DEG, Config.LEAD_AIM_MAX_OFFSET_DEG);
        }

        double effectiveYawErrorDeg = tagInView ? tag.yawErrorDeg - leadOffsetDeg : 0.0;
        boolean yawAligned = !tagInView
                || Math.abs(effectiveYawErrorDeg) <= Config.ALIGN_YAW_TOLERANCE_DEG;

        // Safety: if positional error keeps increasing, stop and ask for axis/direction checks.
        if (tagInView) {
            if (alignStartErr < 0.0) alignStartErr = distance;
            alignMinErr = Math.min(alignMinErr, distance);
            if (now - alignStartMs > 600 && distance > alignMinErr + Config.ALIGN_DIVERGE_IN) {
                cancelAssist("MOVING AWAY FROM TAG - CHECK TAG AXES / NEGATE SETTINGS");
                return;
            }
            if (now - alignStartMs > 1500 && alignMinErr > alignStartErr - 1.0) {
                cancelAssist("NOT GETTING CLOSER - CHECK TAG AXES / TARGET OFFSETS");
                return;
            }
        }

        if (distance <= tolerance && yawAligned) {
            cancelAssist("ALIGNED TO TAG " + (tagInView ? tag.tagId : "(blind finish)"));
            return;
        }

        double forward = 0.0;
        double strafeRight = 0.0;
        if (distance > tolerance) {
            double speed = clip(distance * Config.ALIGN_KP, 0.0, Config.ALIGN_MAX_SPEED);
            speed = Math.max(speed, Config.ALIGN_MIN_SPEED);
            forward = speed * errForward / distance;
            strafeRight = speed * errRight / distance;
        }

        double turn = 0.0;
        if (tagInView && !yawAligned) {
            turn = clip(-effectiveYawErrorDeg * Config.ALIGN_YAW_KP,
                    -Config.ALIGN_MAX_TURN, Config.ALIGN_MAX_TURN);
        } else if (Config.ALIGN_HOLD_HEADING && pinpoint != null) {
            double headingError = normalizeDegrees(alignHoldHeadingDeg - robotHeadingDeg);
            turn = clip(-headingError * Config.ALIGN_HEADING_KP,
                    -Config.ALIGN_MAX_HEADING_TURN, Config.ALIGN_MAX_HEADING_TURN);
        }

        setRobotDrive(forward, strafeRight, turn);
        alignDebug = String.format(Locale.US,
                "to go: fwd %.1f in, right %.1f in | yaw %+.1f° | lead %+.1f° | cmd %.2f / %.2f / %.2f%s",
                errForward, errRight, tagInView ? tag.yawErrorDeg : 0.0,
                leadOffsetDeg, forward, strafeRight, turn, tagInView ? "" : " (tag out of view)");
        assistStatus = String.format(Locale.US, "ALIGNING %s: %.1f in, yaw %+.1f°",
                tagInView ? "TAG " + tag.tagId : "(blind)", distance,
                tagInView ? effectiveYawErrorDeg : 0.0);
    }

    private void alignLastErrForward(double errForward, double errRight, double robotX, double robotY) {
        alignLastErrF = errForward;
        alignLastErrR = errRight;
        alignLastSeenX = robotX;
        alignLastSeenY = robotY;
    }

    // =====================================================================
    // YELLOW BALL APPROACH
    // =====================================================================

    private void runBallApproach(BallTarget target, long now, double robotHeadingDeg) {
        if (now - ballStartMs > Config.BALL_TIMEOUT_MS) {
            cancelAssist("BALL APPROACH TIMEOUT");
            return;
        }
        if (target == null) {
            if (ballSeen && Config.BALL_BLIND_PUSH_MS > 0 && lastBallY >= Config.BALL_BLIND_MIN_Y) {
                if (ballBlindStartMs == 0) ballBlindStartMs = now;
                if (now - ballBlindStartMs <= Config.BALL_BLIND_PUSH_MS) {
                    double blind = Config.BALL_CAMERA_ON_BACK ? -Config.MIN_BALL_FORWARD : Config.MIN_BALL_FORWARD;
                    setRobotDrive(blind, 0.0, 0.0);
                    assistStatus = "FINAL PUSH";
                } else {
                    cancelAssist("AT BALLS (final push)");
                }
                return;
            }
            stopDrive();
            long timeoutMs = ballSeen ? 700 : 3000;
            assistStatus = ballSeen ? "BALLS LOST - SEARCHING" : "NO BALLS YET - POINT CAMERA AT THEM";
            if (now - lastBlobSeenMs > timeoutMs) {
                cancelAssist(ballSeen ? "BALLS LOST" : "NO BALLS FOUND");
            }
            return;
        }
        ballSeen = true;
        lastBallY = target.yFraction;
        ballBlindStartMs = 0;
        lastBlobSeenMs = now;

        double absTx = Math.abs(target.txDeg);
        boolean centered = absTx <= Config.BALL_CENTER_TOLERANCE_DEG;
        double yErr = Config.BALL_STOP_Y_FRACTION - target.yFraction;
        boolean closeEnough = yErr <= 0.0;

        if (centered && closeEnough) {
            cancelAssist("AT BALLS (" + target.count + " in group)");
            return;
        }

        // Smooth strafe ramping near centering tolerance zone
        double strafe = 0.0;
        if (!centered) {
            double propStrafe = absTx * Config.BALL_STRAFE_KP;
            double mag = clip(propStrafe, Config.BALL_MIN_STRAFE, Config.BALL_MAX_STRAFE);
            double rampDeg = 5.0; // ramp down smoothly within 5 deg of tolerance
            if (absTx - Config.BALL_CENTER_TOLERANCE_DEG < rampDeg) {
                double ramp = Math.max(0.0, (absTx - Config.BALL_CENTER_TOLERANCE_DEG) / rampDeg);
                mag = Math.max(Config.BALL_MIN_STRAFE * ramp, mag * ramp);
            }
            strafe = Math.copySign(mag, target.txDeg);
        }

        // Smooth forward drive ramping near stop height
        double drive = 0.0;
        if (!closeEnough) {
            double propDrive = yErr * Config.BALL_FORWARD_KP;
            double mag = clip(propDrive, Config.MIN_BALL_FORWARD, Config.BALL_MAX_SPEED);
            double rampDist = 0.15; // ramp down smoothly over the last 15% of image height
            if (yErr < rampDist) {
                double ramp = Math.max(0.0, yErr / rampDist);
                mag = Math.max(Config.MIN_BALL_FORWARD * ramp, mag * ramp);
            }
            drive = mag;
        }

        double forward = Config.BALL_CAMERA_ON_BACK ? -drive : drive;
        double strafeRight = Config.BALL_CAMERA_ON_BACK ? -strafe : strafe;
        if (Config.REVERSE_BALL_STRAFE) strafeRight = -strafeRight;

        double turn = 0.0;
        if (Config.BALL_HOLD_HEADING && pinpoint != null) {
            double headingError = normalizeDegrees(ballHoldHeadingDeg - robotHeadingDeg);
            turn = clip(-headingError * Config.BALL_HEADING_KP, -Config.BALL_MAX_TURN, Config.BALL_MAX_TURN);
        }

        setRobotDrive(forward, strafeRight, turn);
        assistStatus = String.format(Locale.US, "BALLS: %d in group, tx %.1f°, height %.2f (stop at %.2f)",
                target.count, target.txDeg, target.yFraction, Config.BALL_STOP_Y_FRACTION);
    }

    // =====================================================================
    // TELEMETRY
    // =====================================================================

    private void updateInitTelemetry() {
        double volts = getBatteryVoltage(System.currentTimeMillis());
        String voltStatus = volts < 11.5 ? "(CRITICAL LOW BATTERY!)" : (volts < 12.0 ? "(LOW BATTERY)" : "(GOOD)");

        telemetry.addLine("=== BIOBUZZ TELEOP (DEBUG) ===");
        telemetry.addData("Battery Voltage", "%.2f V %s", volts, voltStatus);
        telemetry.addData("Alliance", Config.RED_ALLIANCE ? "RED (cell tags 30-37)" : "BLUE (cell tags 38-45)");
        telemetry.addData("Drive mode", Config.FIELD_CENTRIC ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Tag Approach + Yaw Align (Cross/A)", Config.ENABLE_APRILTAG_AUTO_ALIGN ? "ENABLED" : "DISABLED");
        telemetry.addData("Ball Approach (B)", Config.ENABLE_BLOB_AUTO_APPROACH ? "ENABLED" : "DISABLED");
        telemetry.addData("Drive motors found", "FL %s | FR %s | BL %s | BR %s",
                frontLeft != null ? "yes" : "NO", frontRight != null ? "yes" : "NO",
                backLeft != null ? "yes" : "NO", backRight != null ? "yes" : "NO");

        telemetry.addLine("--- SENSORS & ACTUATORS ---");
        telemetry.addData("Limelight", limelight != null && limelight.isConnected() ? "CONNECTED" : "DISCONNECTED");
        telemetry.addData("Webcam", webcamStatus);
        telemetry.addData("RGB light", rgbLight != null ? "FOUND" : "NOT FOUND (check config name)");
        telemetry.addData("Pinpoint", pinpoint != null
                ? String.format(Locale.US, "X %.1f, Y %.1f, H %.1f°",
                pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                headingDeg())
                : "DISCONNECTED");
        telemetry.addData("Shooter", shooter != null
                ? String.format(Locale.US, "%s (Name: '%s') | Base: %.0f%% -> Bump: %.0f%% (%dms delay) | AutoSpeed: %s | VoltComp: %s",
                Config.SHOOTER_ENABLED ? "ENABLED" : "DISABLED", Config.SHOOTER_NAME,
                Config.SHOOTER_BASE_POWER * 100.0, Config.SHOOTER_BUMP_POWER * 100.0,
                Config.SHOOTER_BUMP_DELAY_MS,
                Config.SHOOTER_AUTO_SPEED_ENABLED ? "ON" : "OFF",
                Config.SHOOTER_VOLTAGE_COMP_ENABLED ? "ON" : "OFF")
                : "NOT FOUND (Name: '" + Config.SHOOTER_NAME + "')");
        telemetry.addData("Custom Servo", customServo != null
                ? String.format(Locale.US, "%s (Name: '%s') | Timer: %s (%dms)",
                Config.SERVO_ENABLED ? "ENABLED" : "DISABLED", Config.SERVO_NAME,
                Config.SERVO_AUTO_TIMER_ENABLED ? "AUTO" : "MANUAL", Config.SERVO_TIMER_HOLD_MS)
                : "NOT FOUND (Name: '" + Config.SERVO_NAME + "')");

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("[L2] Intake IN | [L1] Intake OUT | [R2] Shooter (Auto Distance Speed)");
        telemetry.addLine("[Square] Toggle Alliance (Init) / Feeder Servo (TeleOp)");
        telemetry.addLine("[Cross] Tag Approach + Yaw Align | [Circle] Go to balls | [D-Pad Up/Down] Servo Pos 1/2");
        telemetry.addLine("[Triangle / Options] Re-zero | [D-Pad Left] Field/Robot | [R1 + D-Pad] Wheel test");
        telemetry.update();
    }

    private void updateTelemetry(double robotX, double robotY, double robotHeadingDeg,
                                 TagView tag, BallTarget ball) {
        double volts = getBatteryVoltage(System.currentTimeMillis());
        String voltStatus = volts < 11.5 ? "(CRITICAL LOW BATTERY!)" : (volts < 12.0 ? "(LOW BATTERY)" : "");

        long timerRemainingMs = (Config.SERVO_AUTO_TIMER_ENABLED && servoTimerActive && !servoStatePos1)
                ? Math.max(0, Config.SERVO_TIMER_HOLD_MS - (System.currentTimeMillis() - servoPos2StartMs))
                : 0;

        boolean bumpActive = Config.SHOOTER_BUMP_ENABLED && !servoStatePos1 && servoPos2StartMs > 0
                && (System.currentTimeMillis() - servoPos2StartMs >= Config.SHOOTER_BUMP_DELAY_MS)
                && (Config.SHOOTER_BUMP_DURATION_MS <= 0 || (System.currentTimeMillis() - servoPos2StartMs < Config.SHOOTER_BUMP_DELAY_MS + Config.SHOOTER_BUMP_DURATION_MS));

        telemetry.addData("Mode", assistStatus);
        telemetry.addData("Battery", "%.2f V %s", volts, voltStatus);
        telemetry.addData("Drive", !Config.FIELD_CENTRIC ? "ROBOT-CENTRIC"
                : (pinpoint == null ? "FIELD-CENTRIC (NO PINPOINT - acting robot-centric)" : "FIELD-CENTRIC"));
        telemetry.addData("Heading vs Zero", "%.1f°", normalizeDegrees(robotHeadingDeg - fieldHeadingOffsetDeg));
        telemetry.addData("Alliance", "%s", lockedRed ? "RED" : "BLUE");
        telemetry.addData("Pose", "X %5.1f | Y %5.1f | H %5.1f°", robotX, robotY, robotHeadingDeg);
        telemetry.addData("Lead Aiming", Config.LEAD_AIM_ENABLED
                ? String.format(Locale.US, "ENABLED | Gain: %+.2f deg/(in/s) | Max: %.1f° | Dz: %.1f in/s",
                Config.LEAD_AIM_GAIN_DEG_PER_IPS, Config.LEAD_AIM_MAX_OFFSET_DEG, Config.LEAD_AIM_VELOCITY_DEADZONE_IPS)
                : "DISABLED");
        telemetry.addData("Shooter", shooter != null
                ? (currentShooterPower > 0
                ? String.format(Locale.US, "ON (%.0f%% power %s%s| target dist: %.1f in)",
                currentShooterPower * 100.0,
                bumpActive ? "[BUMPED 95%] " : "",
                Config.SHOOTER_AUTO_SPEED_ENABLED ? "[AUTO-DIST] " : "",
                latestTargetDistanceIn)
                : String.format(Locale.US, "OFF (Last Target Dist: %.1f in)", latestTargetDistanceIn))
                : "None");
        telemetry.addData("Servo", customServo != null
                ? String.format(Locale.US, "%s %s| Target %.1f° | Actual %.1f°",
                servoStatePos1 ? "POS 1" : "POS 2",
                (timerRemainingMs > 0 ? String.format(Locale.US, "(Timer: %.1fs) ", timerRemainingMs / 1000.0) : ""),
                targetServoAngleDeg, currentServoAngleDeg)
                : "None");
        telemetry.addData("Tag", tag != null
                ? String.format(Locale.US, "#%d (dist %.1f in, tx %+.1f°)",
                tag.tagId, tag.distanceIn, tag.yawErrorDeg)
                : "None");
        telemetry.addData("Tag Debug", tagDebug);
        telemetry.addData("Align Debug", alignDebug);
        telemetry.addData("Balls", ball != null
                ? String.format(Locale.US, "%d in group, tx %+.1f°, height %.2f",
                ball.count, ball.txDeg, ball.yFraction)
                : "None");
        telemetry.update();
    }

    // =====================================================================
    // UTILITIES & WEBCAM PIPELINE
    // =====================================================================

    private static double clip(double val, double low, double high) {
        return Math.max(low, Math.min(high, val));
    }

    private static double normalizeDegrees(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // ----- yellow ball detection -----

    private static class BallBlob {
        double cx, cy, left, right, top, bottom, radius;
        int count;
    }

    private static class BallTarget {
        final int count;
        final double txDeg;
        final double yFraction;

        BallTarget(int count, double txDeg, double yFraction) {
            this.count = count;
            this.txDeg = txDeg;
            this.yFraction = yFraction;
        }
    }

    private static int findRoot(int[] parent, int i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    }

    private static boolean blobsNear(BallBlob a, BallBlob b) {
        double gapX = Math.max(0.0, Math.max(a.left, b.left) - Math.min(a.right, b.right));
        double gapY = Math.max(0.0, Math.max(a.top, b.top) - Math.min(a.bottom, b.bottom));
        double limit = Config.CLUSTER_DIST_DIAMETERS * 2.0 * Math.max(a.radius, b.radius);
        return Math.hypot(gapX, gapY) <= limit;
    }

    private class BallPipeline extends OpenCvPipeline {
        private final Scalar COLOR_GREEN = new Scalar(0, 255, 0);
        private final Scalar COLOR_YELLOW = new Scalar(255, 255, 0);
        private final Scalar COLOR_BLUE = new Scalar(255, 0, 0);
        private final Scalar COLOR_WHITE = new Scalar(255);

        // Pre-allocated Point instances for zero-allocation OpenCV overlay drawing
        private final Point ptRect1 = new Point();
        private final Point ptRect2 = new Point();
        private final Point ptCircle = new Point();
        private final Point ptText = new Point();

        private final Mat small = new Mat(), blurred = new Mat(), rgb = new Mat(), hsv = new Mat();
        private final Mat mask = new Mat(), dist = new Mat(), hierarchy = new Mat(), noHierarchy = new Mat();
        private final Mat maskUp = new Mat();
        private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));
        private final List<MatOfPoint> contours = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {
            try {
                final double scale = input.cols() > 400 ? 0.5 : 1.0;
                Imgproc.resize(input, small, new org.opencv.core.Size(input.cols() * scale, input.rows() * scale),
                        0, 0, Imgproc.INTER_AREA);
                Imgproc.GaussianBlur(small, blurred, new org.opencv.core.Size(3, 3), 0);
                Imgproc.cvtColor(blurred, rgb, Imgproc.COLOR_RGBA2RGB);
                Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
                Core.inRange(hsv,
                        new Scalar(Config.HSV_H_MIN, Config.HSV_S_MIN, Config.HSV_V_MIN),
                        new Scalar(Config.HSV_H_MAX, Config.HSV_S_MAX, Config.HSV_V_MAX), mask);
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
                Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
                Imgproc.distanceTransform(mask, dist, Imgproc.DIST_L2, 3);

                contours.clear();
                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                List<BallBlob> blobs = new ArrayList<>();
                double frameArea = small.cols() * (double) small.rows();
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area >= Config.MIN_BALL_AREA_PX && area <= 0.3 * frameArea) {
                        Rect rect = Imgproc.boundingRect(contour);

                        Mat roiDist = null;
                        Mat roiMask = null;
                        try {
                            roiDist = dist.submat(rect);
                            roiMask = Mat.zeros(rect.size(), CvType.CV_8UC1);
                            ptRect1.x = -rect.x;
                            ptRect1.y = -rect.y;
                            Imgproc.drawContours(roiMask, Collections.singletonList(contour), 0, COLOR_WHITE, -1,
                                    Imgproc.LINE_8, noHierarchy, 0, ptRect1);
                            Core.MinMaxLocResult mm = Core.minMaxLoc(roiDist, roiMask);

                            double radius = Math.max(1.5, mm.maxVal);
                            BallBlob blob = new BallBlob();
                            blob.radius = radius;
                            blob.count = (int) Math.max(1, Math.round(area / (Math.PI * radius * radius)));
                            blob.left = rect.x;
                            blob.right = rect.x + rect.width;
                            blob.top = rect.y;
                            blob.bottom = rect.y + rect.height;
                            blob.cx = rect.x + rect.width / 2.0;
                            blob.cy = rect.y + rect.height / 2.0;
                            blobs.add(blob);
                        } finally {
                            if (roiDist != null) roiDist.release();
                            if (roiMask != null) roiMask.release();
                        }
                    }
                    contour.release();
                }
                contours.clear();

                if (Config.SHOW_BALL_MASK) {
                    Imgproc.resize(mask, maskUp, input.size());
                    Imgproc.cvtColor(maskUp, input, Imgproc.COLOR_GRAY2RGBA);
                }

                int m = blobs.size();
                if (m == 0) {
                    latestBalls = null;
                    return input;
                }

                int[] parent = new int[m];
                for (int i = 0; i < m; i++) parent[i] = i;
                for (int i = 0; i < m; i++) {
                    for (int j = i + 1; j < m; j++) {
                        if (blobsNear(blobs.get(i), blobs.get(j))) {
                            int ri = findRoot(parent, i), rj = findRoot(parent, j);
                            if (ri != rj) parent[ri] = rj;
                        }
                    }
                }

                int capacity = Math.max(1, Config.BALL_CAPACITY);
                int[] groupCount = new int[m];
                double[] bottom = new double[m];
                for (int i = 0; i < m; i++) {
                    BallBlob b = blobs.get(i);
                    int root = findRoot(parent, i);
                    groupCount[root] += b.count;
                    bottom[root] = Math.max(bottom[root], b.bottom);
                }

                int best = -1;
                for (int i = 0; i < m; i++) {
                    if (groupCount[i] == 0) continue;
                    int score = Math.min(groupCount[i], capacity);
                    int bestScore = best < 0 ? 0 : Math.min(groupCount[best], capacity);
                    if (best < 0 || score > bestScore || (score == bestScore && bottom[i] > bottom[best])) {
                        best = i;
                    }
                }

                List<BallBlob> members = new ArrayList<>();
                for (int i = 0; i < m; i++) {
                    if (findRoot(parent, i) == best) members.add(blobs.get(i));
                }
                members.sort((p1, p2) -> Double.compare(p2.bottom, p1.bottom));
                double sumX = 0.0;
                double taken = 0.0;
                for (BallBlob mb : members) {
                    if (taken >= capacity) break;
                    sumX += mb.cx * mb.count;
                    taken += mb.count;
                }
                double centerX = sumX / taken;
                double tx = ((centerX / small.cols()) - 0.5) * Config.WEBCAM_HFOV_DEG;
                double yFraction = bottom[best] / small.rows();
                latestBalls = new BallTarget(groupCount[best], tx, yFraction);
                latestBallsMs = System.currentTimeMillis();

                for (int i = 0; i < m; i++) {
                    BallBlob b = blobs.get(i);
                    boolean inBest = findRoot(parent, i) == best;
                    ptRect1.x = b.left / scale;
                    ptRect1.y = b.top / scale;
                    ptRect2.x = b.right / scale;
                    ptRect2.y = b.bottom / scale;
                    Imgproc.rectangle(input, ptRect1, ptRect2, inBest ? COLOR_GREEN : COLOR_YELLOW, 2);
                }
                ptCircle.x = centerX / scale;
                ptCircle.y = (bottom[best] - 10) / scale;
                Imgproc.circle(input, ptCircle, 10, COLOR_BLUE, 3);

                ptText.x = centerX / scale + 14;
                ptText.y = (bottom[best] - 10) / scale;
                Imgproc.putText(input, String.valueOf(groupCount[best]), ptText,
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, COLOR_BLUE, 2);
            } catch (Exception e) {
                latestBalls = null;
            }
            return input;
        }
    }
}