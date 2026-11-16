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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * BIOBUZZ TeleOp (Debug Version with Enhanced Limelight AprilTag Localization)
 *
 * Controls (Gamepad 1):
 *  - Sticks: field-centric mecanum drive
 *  - Y / Options: re-zero field heading
 *  - A: AprilTag align (Limelight): drives right UNDER the closest tag from Config.ALIGN_TAG_IDS (toggle, sticks cancel it)
 *  - B: drive to the biggest group of yellow balls seen by Webcam 1 (toggle, sticks cancel it)
 *  - X (during init): switch RED / BLUE alliance
 *  - RB + D-Pad: single-wheel direction test
 *
 * Limelight setup: AprilTag pipeline, family 36h11, tag size 82.5 mm (3.25 in), and the camera position
 * entered on the Limelight settings page so tag positions are measured from the robot's center.
 */
@TeleOp(name = "BIOBUZZ TeleOp (Debug)", group = "BIOBUZZ")
public class BASE_Debug extends LinearOpMode {

    @com.acmerobotics.dashboard.config.Config
    public static class Config {
        // ---------- Hardware names (must match the Robot Configuration) ----------
        public static String FRONT_LEFT_NAME = "front_left";
        public static String FRONT_RIGHT_NAME = "back_right";
        public static String BACK_LEFT_NAME = "back_left";
        public static String BACK_RIGHT_NAME = "front_right";
        public static String PINPOINT_NAME = "pinpoint";
        public static String LIMELIGHT_NAME = "limelight";
        public static String WEBCAM_NAME = "Webcam 1";
        public static String RGB_LIGHT_NAME = "rgb_light";   // goBILDA RGB Indicator Light (set up as a plain "Servo")

        // ---------- RGB indicator light color (servo position) ----------
        // Off 0.0 | Red 0.277 | Orange 0.333 | Yellow 0.388 | Sage 0.444 | Green 0.500
        // Azure 0.555 | Blue 0.611 | Indigo 0.666 | Violet 0.722 | White 1.0
        public static double RGB_LIGHT_POSITION = 0.555;      // approximate #4BB4CB (blue-cyan)

        // ---------- Motor reversals (use RB + D-Pad test to check) ----------
        public static boolean REVERSE_FRONT_LEFT = true;
        public static boolean REVERSE_FRONT_RIGHT = false;
        public static boolean REVERSE_BACK_LEFT = true;
        public static boolean REVERSE_BACK_RIGHT = false;

        // ---------- Drive direction ----------
        public static boolean INVERT_FORWARD = false; // true = the opposite end is the robot's front (also flips strafe)
        public static boolean REVERSE_DRIVE_FORWARD = false; // correction: flip forward / backward commands
        public static boolean REVERSE_DRIVE_TURN = false;    // correction: flip clockwise / counter-clockwise commands

        // ---------- Field-centric ("swerve style") driving ----------
        public static boolean FIELD_CENTRIC = true;          // true = swerve style, false = normal robot-centric
        public static boolean REVERSE_HEADING = false;       // flip if the robot drives the wrong way after you turn it
        public static boolean REVERSE_STRAFE_STICK = false;  // flip if left stick left/right is backwards

        // ---------- Pinpoint ----------
        public static double PINPOINT_X_POD_OFFSET_MM = 0.0; // measure with a ruler
        public static double PINPOINT_Y_POD_OFFSET_MM = 0.0; // measure with a ruler
        public static boolean REVERSE_X_ENCODER = false;
        public static boolean REVERSE_Y_ENCODER = false;

        // ---------- Webcam exposure ----------
        public static boolean WEBCAM_MANUAL_EXPOSURE = false;
        public static long WEBCAM_EXPOSURE_MS = 15;
        public static int WEBCAM_GAIN = 100;

        // ---------- Match setup ----------
        public static boolean RED_ALLIANCE = true;
        public static boolean ENABLE_APRILTAG_AUTO_ALIGN = true;
        public static boolean ENABLE_BLOB_AUTO_APPROACH = true;

        // ---------- Start pose ----------
        public static double START_X_IN = 0.0;
        public static double START_Y_IN = 0.0;
        public static double START_HEADING_DEG = 0.0;

        // ---------- Driver / assist limits ----------
        public static double STICK_DEADZONE = 0.16;

        // ---------- AprilTag align: drive right UNDER the tag (Limelight) ----------
        // The Limelight's position on the robot is set on the Limelight settings page, so the tag position it
        // reports is measured from the robot's center.
        public static int APRILTAG_PIPELINE = 0;              // Limelight pipeline number for AprilTags

        public static String RED_ALIGN_TAG_IDS = "31,35";    // tags to align to on RED
        public static String BLUE_ALIGN_TAG_IDS = "39,43";   // tags to align to on BLUE

        // Pose space mode: true = getTargetPoseRobotSpace() [X=fwd, Y=right, Z=up], false = getTargetPoseCameraSpace() [X=right, Y=down, Z=fwd]
        public static boolean USE_ROBOT_SPACE_POSE = true;

        // Axis selection (0 = X, 1 = Y, 2 = Z).
        // For Robot Space: Forward is Axis 0 (X), Right is Axis 1 (Y).
        // For Camera Space: Forward is Axis 2 (Z), Right is Axis 0 (X).
        public static int TAG_FORWARD_AXIS = 0;
        public static int TAG_RIGHT_AXIS = 1;
        public static boolean TAG_FORWARD_NEGATE = false;
        public static boolean TAG_RIGHT_NEGATE = false;        // flip so right is + when the tag is to the robot's RIGHT

        // Where the tag should end up, measured from the robot's center. 0 and 0 = exactly overhead.
        public static double ALIGN_TARGET_FORWARD_IN = 50.0;   // + = tag ends up ahead of center
        public static double ALIGN_TARGET_RIGHT_IN = 40.5;     // + = tag ends up to the right of center

        public static boolean BLUE_MIRRORS_SIDE = true;        // blue puts the tag on the opposite side from red
        public static double ALIGN_TOLERANCE_IN = 1.5;        // done when within this many inches

        public static double ALIGN_KP = 0.04;                 // speed per inch of error
        public static double ALIGN_MAX_SPEED = 0.50;
        public static double ALIGN_MIN_SPEED = 0.15;
        public static boolean ALIGN_HOLD_HEADING = true;      // keep the robot facing the same way while it slides under (needs Pinpoint)
        public static double ALIGN_HEADING_KP = 0.01;
        public static double ALIGN_MAX_TURN = 0.15;
        public static double ALIGN_BLIND_MAX_IN = 30.0;       // if the tag leaves the view closer than this, finish the move with the Pinpoint
        public static double ALIGN_DIVERGE_IN = 8.0;          // safety: stops if it gets this much farther from the tag
        public static long ALIGN_TIMEOUT_MS = 6000;           // auto align stops by itself after this long

        // ---------- Yellow ball detection (Logitech webcam, "Webcam 1") ----------
        // HSV colors: tune on the Dashboard camera stream. Turn SHOW_BALL_MASK on to see what counts as yellow.
        public static int HSV_H_MIN = 20;
        public static int HSV_H_MAX = 38;
        public static int HSV_S_MIN = 80;
        public static int HSV_S_MAX = 255;
        public static int HSV_V_MIN = 60;
        public static int HSV_V_MAX = 255;
        public static boolean SHOW_BALL_MASK = false;       // true = camera stream shows white where it sees yellow
        public static double MIN_BALL_AREA_PX = 40.0;       // smallest yellow blob that counts (in a 320x240 image)
        public static double CLUSTER_DIST_DIAMETERS = 5.0;  // balls closer than this many ball-widths = same group
        public static double WEBCAM_HFOV_DEG = 60.0;        // webcam field of view (C270 is about 60)

        // ---------- Ball approach (B button) ----------
        public static boolean BALL_CAMERA_ON_BACK = false;  // false = camera is on the robot's front (it is now). true = camera is on the back
        public static boolean REVERSE_BALL_STRAFE = false;  // flip if it slides AWAY from the balls sideways
        public static double BALL_STOP_Y_FRACTION = 0.97;   // stops when the group's bottom reaches this height in the image (0 = top, 1 = bottom)
        public static double BALL_FORWARD_KP = 4.0;         // drive power per image-height error
        public static double BALL_STRAFE_KP = 0.04;         // sideways power per degree off-center
        public static double BALL_CENTER_TOLERANCE_DEG = 3.0;
        public static double BALL_MAX_SPEED = 1.00;         // top driving speed toward the balls
        public static double MIN_BALL_FORWARD = 0.40;       // slowest driving speed toward the balls
        public static double BALL_MAX_STRAFE = 0.80;        // top sideways speed toward the balls
        public static double BALL_MIN_STRAFE = 0.30;        // slowest sideways speed toward the balls
        public static boolean BALL_HOLD_HEADING = true;     // keeps the robot facing the same way while it slides in (needs Pinpoint)
        public static double BALL_HEADING_KP = 0.01;        // turn power per degree of heading drift
        public static double BALL_MAX_TURN = 0.15;          // cap on the heading-hold turn
        public static int BALL_CAPACITY = 4;                // most balls the robot can hold. A group counts as at most this many
        public static long BALL_BLIND_PUSH_MS = 300;        // if the balls slip out of view at the bottom, keep driving this long (0 = off)
        public static double BALL_BLIND_MIN_Y = 0.85;       // only do the final push if the balls were at least this low in the image

        // ---------- Intake ----------
        public static String INTAKE_NAME = "intake";
        public static boolean REVERSE_INTAKE = false;
        public static double INTAKE_IN_POWER = 0.5;
        public static double INTAKE_OUT_POWER = 1.0;
        public static double TRIGGER_THRESHOLD = 0.1;

        public static long BALL_TIMEOUT_MS = 6000;          // stops by itself after this long
    }

    // ---------- Hardware ----------
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private DcMotor intake;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private OpenCvWebcam webcam;
    private VoltageSensor batteryVoltageSensor;
    private Servo rgbLight;
    private double lastRgbPosition = -1.0;

    // ---------- Webcam ball detection ----------
    private final BallPipeline ballPipeline = new BallPipeline();
    private volatile BallTarget latestBalls;
    private volatile long latestBallsMs = 0;
    private volatile String webcamStatus = "NOT INITIALIZED";

    // ---------- State ----------
    private boolean autoAlignActive = false;
    private boolean blobApproachActive = false;
    private boolean previousA = false, previousB = false, previousY = false;
    private boolean previousOptions = false, previousInitX = false, previousLeftBumper = false;

    private double fieldHeadingOffsetDeg = 0.0;
    private double lastXOffset = Double.NaN, lastYOffset = Double.NaN;
    private boolean lastXEncoderRev = false, lastYEncoderRev = false;

    private boolean lastManualExposure = false;
    private long lastExposureMs = -1;
    private int lastGain = -1;

    private String assistStatus = "READY";
    private long statusHoldUntilMs = 0;
    private String tagDebug = "";
    private boolean alignTagSeen = false;
    private int lockedAlignTagId = -1;
    private double alignHoldHeadingDeg = 0.0;
    private double alignLastErrF = 0.0, alignLastErrR = 0.0;
    private double alignLastSeenX = 0.0, alignLastSeenY = 0.0;
    private double alignTargetFieldX = 0.0, alignTargetFieldY = 0.0;
    private double alignMinErr = Double.MAX_VALUE;
    private double alignStartErr = -1.0;
    private long alignStartMs = 0;
    private boolean ballSeen = false;
    private double ballHoldHeadingDeg = 0.0;
    private double lastBallY = 0.0;
    private long ballBlindStartMs = 0;
    private long ballStartMs = 0;
    private String alignDebug = "";

    private boolean allianceLocked = false;
    private boolean lockedRed = true;

    private String activeTagIds() {
        boolean red = allianceLocked ? lockedRed : Config.RED_ALLIANCE;
        return red ? Config.RED_ALIGN_TAG_IDS : Config.BLUE_ALIGN_TAG_IDS;
    }
    private double targetRightIn() {
        boolean red = allianceLocked ? lockedRed : Config.RED_ALLIANCE;
        return (Config.BLUE_MIRRORS_SIDE && !red) ? -Config.ALIGN_TARGET_RIGHT_IN : Config.ALIGN_TARGET_RIGHT_IN;
    }
    private long lastAlignTagSeenMs = 0, lastBlobSeenMs = 0;

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
        setIndicatorLightPosition(RAINBOW_POSITIONS[0]);
        configurePinpoint();

        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(Config.APRILTAG_PIPELINE);
            limelight.start();
        }
        initializeWebcam();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        fieldHeadingOffsetDeg = Config.START_HEADING_DEG;

        // ---------- INIT LOOP ----------
        long initLightStartMs = System.currentTimeMillis();
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !previousInitX) {
                Config.RED_ALLIANCE = !Config.RED_ALLIANCE;
            }
            previousInitX = gamepad1.x;

            if (gamepad1.dpad_left && !previousLeftBumper) {
                Config.FIELD_CENTRIC = !Config.FIELD_CENTRIC;
            }
            previousLeftBumper = gamepad1.dpad_left;

            updatePinpointConfigIfNeeded();
            updateWebcamExposureIfNeeded();
            updateInitIndicatorLight(System.currentTimeMillis() - initLightStartMs);
            if (pinpoint != null) {
                pinpoint.update();
                // The way the robot is facing when you press START becomes "forward" for the driver
                fieldHeadingOffsetDeg = headingDeg();
            }

            updateInitTelemetry();
            sleep(20);
        }

        lockedRed = Config.RED_ALLIANCE;
        allianceLocked = true;

        // ---------- MAIN LOOP ----------
        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            updatePinpointConfigIfNeeded();
            updateWebcamExposureIfNeeded();
            updateTeleOpIndicatorLight();

            double robotX = 0.0, robotY = 0.0, robotHeadingDeg = 0.0;
            if (pinpoint != null) {
                pinpoint.update();
                robotX = pinpoint.getPosX(DistanceUnit.INCH);
                robotY = pinpoint.getPosY(DistanceUnit.INCH);
                robotHeadingDeg = headingDeg();
            }

            handleButtons(now);

            runIntake();

            boolean manualOverride = hasManualStickInput();
            if (manualOverride && (autoAlignActive || blobApproachActive)) {
                cancelAssist("MANUAL OVERRIDE");
            }

            TagView tagView = readAprilTags();
            BallTarget ballTarget = (latestBalls != null && now - latestBallsMs <= 700)
                    ? latestBalls : null;

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
        if (limelight != null) limelight.stop();
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    // =====================================================================
    // SETUP
    // =====================================================================

    // Rainbow order, taken from the goBILDA position table
    private static final double[] RAINBOW_POSITIONS = {
            0.277,  // Red
            0.333,  // Orange
            0.388,  // Yellow
            0.500,  // Green
            0.611,  // Blue
            0.666,  // Indigo
            0.722   // Violet
    };
    private static final long INIT_LIGHT_COLOR_HOLD_MS = 500;   // time on each color

    /** Applies a light position only when it has changed enough to matter. */
    private void setIndicatorLightPosition(double position) {
        if (rgbLight == null) return;

        double safePosition = clip(position, 0.0, 1.0);
        if (Math.abs(safePosition - lastRgbPosition) >= 0.0001) {
            rgbLight.setPosition(safePosition);
            lastRgbPosition = safePosition;
        }
    }

    private void updateInitIndicatorLight(long elapsedMs) {
        int index = (int) ((elapsedMs / INIT_LIGHT_COLOR_HOLD_MS) % RAINBOW_POSITIONS.length);
        setIndicatorLightPosition(RAINBOW_POSITIONS[index]);
    }

    private void updateTeleOpIndicatorLight() {
        setIndicatorLightPosition(Config.RGB_LIGHT_POSITION);
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
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Config.START_X_IN, Config.START_Y_IN,
                AngleUnit.DEGREES, Config.START_HEADING_DEG));
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

    // =====================================================================
    // INPUT
    // =====================================================================

    private void handleButtons(long now) {
        boolean pressedA = gamepad1.a && !previousA;
        boolean pressedB = gamepad1.b && !previousB;
        boolean pressedY = gamepad1.y && !previousY;
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
                lockedAlignTagId = -1;
                alignHoldHeadingDeg = headingDeg();
                alignMinErr = Double.MAX_VALUE;
                alignStartErr = -1.0;
                alignStartMs = now;
                lastAlignTagSeenMs = now;
                assistStatus = "LOOKING FOR CELL TAGS";
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

        if (pressedY || pressedOptions) {
            if (pinpoint != null) fieldHeadingOffsetDeg = headingDeg();
            assistStatus = "HEADING RE-ZEROED";
        } else if (gamepad1.y || optionsHeld) {
            // keep the message while held, it clears on the next normal loop
        } else if (assistStatus.equals("HEADING RE-ZEROED")) {
            assistStatus = "READY";
        }

        boolean pressedLeftBumper = gamepad1.dpad_left && !previousLeftBumper;
        if (pressedLeftBumper) {
            Config.FIELD_CENTRIC = !Config.FIELD_CENTRIC;
            assistStatus = Config.FIELD_CENTRIC ? "FIELD-CENTRIC ON" : "ROBOT-CENTRIC ON";
        }
        previousLeftBumper = gamepad1.dpad_left;

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousY = gamepad1.y;
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
        lockedAlignTagId = -1;
        assistStatus = reason;
        statusHoldUntilMs = System.currentTimeMillis() + 4000;
        stopDrive();
    }

    // =====================================================================
    // DRIVE
    // =====================================================================

    private boolean runFieldCentricDrive(double robotHeadingDeg) {
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

        double stickForward = Math.abs(gamepad1.left_stick_y) > Config.STICK_DEADZONE ? -gamepad1.left_stick_y : 0.0;
        double stickRight = Math.abs(gamepad1.left_stick_x) > Config.STICK_DEADZONE ? gamepad1.left_stick_x : 0.0;
        if (Config.REVERSE_STRAFE_STICK) stickRight = -stickRight;
        double turn = Math.abs(gamepad1.right_stick_x) > Config.STICK_DEADZONE ? gamepad1.right_stick_x : 0.0;

        if (stickForward == 0.0 && stickRight == 0.0 && turn == 0.0) {
            stopDrive();
            return false;
        }

        double forward = stickForward;
        double strafeRight = stickRight;

        if (Config.FIELD_CENTRIC && pinpoint != null) {
            double rel = Math.toRadians(normalizeDegrees(robotHeadingDeg - fieldHeadingOffsetDeg));
            strafeRight = stickRight * Math.cos(rel) + stickForward * Math.sin(rel);
            forward = -stickRight * Math.sin(rel) + stickForward * Math.cos(rel);
        }

        setRobotDrive(forward, strafeRight, turn);
        return false;
    }

    private void setRobotDrive(double forward, double strafeRight, double turn) {
        if (Config.INVERT_FORWARD) {
            forward = -forward;
            strafeRight = -strafeRight;
        }

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

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.abs(fr)));
        max = Math.max(max, Math.max(Math.abs(bl), Math.abs(br)));

        if (frontLeft != null) frontLeft.setPower(fl / max);
        if (frontRight != null) frontRight.setPower(fr / max);
        if (backLeft != null) backLeft.setPower(bl / max);
        if (backRight != null) backRight.setPower(br / max);
    }

    private void runIntake() {
        if (intake == null) return;
        if (gamepad1.left_bumper) {
            intake.setPower(-Config.INTAKE_OUT_POWER);
        } else if (gamepad1.left_trigger > Config.TRIGGER_THRESHOLD) {
            intake.setPower(Config.INTAKE_IN_POWER);
        } else {
            intake.setPower(0.0);
        }
    }

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }

    // =====================================================================
    // APRILTAG ALIGN (BIOBUZZ CELLS)
    // =====================================================================

    private static class TagView {
        final int tagId;
        final double forwardIn;   // + = tag is toward robot front
        final double rightIn;     // + = tag is to robot right
        final double rawX, rawY, rawZ; // raw 3D pose numbers from Limelight
        final int tagCount;

        TagView(int tagId, double forwardIn, double rightIn, double rawX, double rawY, double rawZ, int tagCount) {
            this.tagId = tagId;
            this.forwardIn = forwardIn;
            this.rightIn = rightIn;
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
            return null;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            tagDebug = "NO RESULT";
            return null;
        }
        if (result.getPipelineIndex() != Config.APRILTAG_PIPELINE) {
            tagDebug = "WRONG PIPELINE: Limelight is on " + result.getPipelineIndex()
                    + ", Config wants " + Config.APRILTAG_PIPELINE;
            return null;
        }
        if (!result.isValid()) {
            tagDebug = "NO TARGET SEEN";
            return null;
        }
        if (limelight.getTimeSinceLastUpdate() > 500) {
            tagDebug = "STALE DATA (" + limelight.getTimeSinceLastUpdate() + "ms)";
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            tagDebug = "NO TAGS IN RESULT";
            return null;
        }

        int forwardAxis = Math.max(0, Math.min(2, Config.TAG_FORWARD_AXIS));
        int rightAxis = Math.max(0, Math.min(2, Config.TAG_RIGHT_AXIS));

        StringBuilder seen = new StringBuilder();
        int allowedCount = 0;
        TagView best = null;
        double bestError = Double.MAX_VALUE;

        // If we previously locked onto a tag ID during alignment, prefer staying locked to it
        LLResultTypes.FiducialResult lockedFiducial = null;
        if (lockedAlignTagId != -1) {
            for (LLResultTypes.FiducialResult tag : fiducials) {
                if (tag.getFiducialId() == lockedAlignTagId) {
                    lockedFiducial = tag;
                    break;
                }
            }
        }

        if (lockedFiducial != null) {
            double[] raw = tagPoseInches(lockedFiducial);
            if (raw != null) {
                allowedCount = 1;
                double forward = raw[forwardAxis] * (Config.TAG_FORWARD_NEGATE ? -1.0 : 1.0);
                double right = raw[rightAxis] * (Config.TAG_RIGHT_NEGATE ? -1.0 : 1.0);
                tagDebug = String.format(Locale.US, "OK - LOCKED Tag #%d", lockedAlignTagId);
                return new TagView(lockedAlignTagId, forward, right, raw[0], raw[1], raw[2], 1);
            }
        }

        // Search among allowed tags for the closest one to target
        for (LLResultTypes.FiducialResult tag : fiducials) {
            seen.append(tag.getFiducialId()).append(' ');
            if (!isAllowedTag(tag.getFiducialId())) continue;
            allowedCount++;

            double[] raw = tagPoseInches(tag);
            if (raw == null) continue;
            double forward = raw[forwardAxis] * (Config.TAG_FORWARD_NEGATE ? -1.0 : 1.0);
            double right = raw[rightAxis] * (Config.TAG_RIGHT_NEGATE ? -1.0 : 1.0);

            double error = Math.hypot(forward - Config.ALIGN_TARGET_FORWARD_IN, right - targetRightIn());
            if (error < bestError) {
                bestError = error;
                best = new TagView(tag.getFiducialId(), forward, right, raw[0], raw[1], raw[2], allowedCount);
            }
        }

        if (best == null) {
            tagDebug = allowedCount == 0
                    ? "SEES TAG IDs [" + seen.toString().trim() + "] BUT NONE IN ALIGN_TAG_IDS (" + activeTagIds() + ")"
                    : "TAG SEEN BUT NO 3D POSE (check Limelight tag size and pose settings)";
            return null;
        }

        if (autoAlignActive && lockedAlignTagId == -1) {
            lockedAlignTagId = best.tagId;
        }

        tagDebug = "OK - IDs [" + seen.toString().trim() + "]";
        return new TagView(best.tagId, best.forwardIn, best.rightIn, best.rawX, best.rawY, best.rawZ, allowedCount);
    }

    /**
     * Drives the robot so the tag ends up right above its center (plus offsets).
     * Uses Limelight visual feedback while visible, and switches to 2D odometry tracking
     * via Pinpoint if tag temporarily leaves camera view.
     */
    private void runAprilTagAlign(TagView tag, long now, double robotX, double robotY, double robotHeadingDeg) {
        if (now - alignStartMs > Config.ALIGN_TIMEOUT_MS) {
            cancelAssist("ALIGN TIMEOUT");
            return;
        }

        double tolerance = Math.max(0.5, Config.ALIGN_TOLERANCE_IN);
        double errForward, errRight;
        boolean tagInView = tag != null;

        if (tagInView) {
            alignTagSeen = true;
            lastAlignTagSeenMs = now;
            errForward = tag.forwardIn - Config.ALIGN_TARGET_FORWARD_IN;
            errRight = tag.rightIn - targetRightIn();

            alignLastErrF = errForward;
            alignLastErrR = errRight;
            alignLastSeenX = robotX;
            alignLastSeenY = robotY;

            // Compute target position in Field coordinates (inches) for accurate odometry-based blind tracking
            double rad = Math.toRadians(robotHeadingDeg);
            alignTargetFieldX = robotX + errForward * Math.cos(rad) - errRight * Math.sin(rad);
            alignTargetFieldY = robotY + errForward * Math.sin(rad) + errRight * Math.cos(rad);
        } else {
            if (!alignTagSeen) {
                stopDrive();
                assistStatus = "NO TAG YET - POINT LIMELIGHT AT TAG";
                if (now - alignStartMs > 3000) cancelAssist("NO TAG FOUND (see Tag Debug)");
                return;
            }
            double lastDistance = Math.hypot(alignLastErrF, alignLastErrR);
            if (pinpoint == null || lastDistance > Config.ALIGN_BLIND_MAX_IN) {
                cancelAssist("TAG LOST");
                return;
            }

            // Tag out of view: use Pinpoint odometry to calculate remaining error in current robot frame
            double dxField = alignTargetFieldX - robotX;
            double dyField = alignTargetFieldY - robotY;
            double rad = Math.toRadians(robotHeadingDeg);
            errForward = dxField * Math.cos(rad) + dyField * Math.sin(rad);
            errRight = -dxField * Math.sin(rad) + dyField * Math.cos(rad);
        }

        double distance = Math.hypot(errForward, errRight);
        if (distance <= tolerance) {
            cancelAssist("ALIGNED UNDER TAG " + (tagInView ? tag.tagId : ""));
            return;
        }

        // Divergence safety check
        if (tagInView) {
            if (alignStartErr < 0.0) alignStartErr = distance;
            alignMinErr = Math.min(alignMinErr, distance);
            if (now - alignStartMs > 600 && distance > alignMinErr + Config.ALIGN_DIVERGE_IN) {
                cancelAssist("MOVING AWAY FROM TAG - check TAG_FORWARD_NEGATE or TAG_RIGHT_NEGATE");
                return;
            }
            // Only enforce progress check if starting distance was significantly larger than tolerance
            if (alignStartErr > tolerance + 4.0 && now - alignStartMs > 1800 && alignMinErr > alignStartErr - 1.0) {
                cancelAssist("NOT GETTING CLOSER - check axis mapping and NEGATE settings");
                return;
            }
        }

        // Proportional speed with smooth ramping near target tolerance zone (eliminates 15% step overshoot)
        double speed;
        if (distance <= tolerance) {
            speed = 0.0;
        } else {
            double propSpeed = distance * Config.ALIGN_KP;
            speed = clip(propSpeed, Config.ALIGN_MIN_SPEED, Config.ALIGN_MAX_SPEED);
            double rampDist = 3.0; // ramp down smoothly over the last 3 inches
            if (distance - tolerance < rampDist) {
                double ramp = (distance - tolerance) / rampDist;
                speed = Math.max(Config.ALIGN_MIN_SPEED * ramp, speed * ramp);
            }
        }

        double forward = speed * errForward / distance;
        double strafeRight = speed * errRight / distance;

        double turn = 0.0;
        if (Config.ALIGN_HOLD_HEADING && pinpoint != null) {
            double headingError = normalizeDegrees(alignHoldHeadingDeg - robotHeadingDeg);
            turn = clip(-headingError * Config.ALIGN_HEADING_KP, -Config.ALIGN_MAX_TURN, Config.ALIGN_MAX_TURN);
        }

        setRobotDrive(forward, strafeRight, turn);
        alignDebug = String.format(Locale.US, "to go: fwd %.1f in, right %.1f in | cmd fwd %.2f, strafe %.2f%s",
                errForward, errRight, forward, strafeRight, tagInView ? "" : " (blind)");
        assistStatus = String.format(Locale.US, "ALIGNING %s: %.1f in to go",
                tagInView ? "TAG " + tag.tagId : "(blind)", distance);
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

        boolean centered = Math.abs(target.txDeg) <= Config.BALL_CENTER_TOLERANCE_DEG;
        double yErr = Config.BALL_STOP_Y_FRACTION - target.yFraction;
        boolean closeEnough = yErr <= 0.0;

        if (centered && closeEnough) {
            cancelAssist("AT BALLS (" + target.count + " in group)");
            return;
        }

        double strafe = 0.0;
        if (!centered) {
            strafe = clip(target.txDeg * Config.BALL_STRAFE_KP, -Config.BALL_MAX_STRAFE, Config.BALL_MAX_STRAFE);
            strafe = Math.copySign(Math.max(Math.abs(strafe), Config.BALL_MIN_STRAFE), strafe);
        }

        double drive = 0.0;
        if (!closeEnough) {
            drive = clip(yErr * Config.BALL_FORWARD_KP, 0.0, Config.BALL_MAX_SPEED);
            drive = Math.max(drive, Config.MIN_BALL_FORWARD);
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
        double volts = batteryVoltageSensor != null ? batteryVoltageSensor.getVoltage() : 0.0;
        telemetry.addLine("=== BIOBUZZ TELEOP (DEBUG) ===");
        telemetry.addData("Battery", "%.2f V %s", volts, volts < 12.0 ? "(LOW!)" : "(GOOD)");
        telemetry.addData("Alliance", Config.RED_ALLIANCE ? "RED (cell tags 30-37)" : "BLUE (cell tags 38-45)");
        telemetry.addData("Drive mode", Config.FIELD_CENTRIC ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Tag Align (A)", Config.ENABLE_APRILTAG_AUTO_ALIGN ? "ENABLED" : "DISABLED");
        telemetry.addData("Ball Approach (B)", Config.ENABLE_BLOB_AUTO_APPROACH ? "ENABLED" : "DISABLED");
        telemetry.addData("Drive motors found", "FL %s | FR %s | BL %s | BR %s",
                frontLeft != null ? "yes" : "NO", frontRight != null ? "yes" : "NO",
                backLeft != null ? "yes" : "NO", backRight != null ? "yes" : "NO");

        telemetry.addLine("--- SENSORS ---");
        telemetry.addData("Limelight", limelight != null && limelight.isConnected() ? "CONNECTED" : "DISCONNECTED");
        telemetry.addData("Webcam", webcamStatus);
        telemetry.addData("RGB light", rgbLight != null ? "FOUND" : "NOT FOUND (check config name)");
        telemetry.addData("Pinpoint", pinpoint != null
                ? String.format(Locale.US, "X %.1f, Y %.1f, H %.1f°",
                pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                headingDeg())
                : "DISCONNECTED");

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("[X] Toggle alliance | [A] Align to cell | [B] Go to balls");
        telemetry.addLine("[Y / Options] Re-zero heading | [D-Pad Left] Field/Robot-centric | [RB + D-Pad] Wheel test");
        telemetry.update();
    }

    private void updateTelemetry(double robotX, double robotY, double robotHeadingDeg,
                                 TagView tag, BallTarget ball) {
        double volts = batteryVoltageSensor != null ? batteryVoltageSensor.getVoltage() : 0.0;
        telemetry.addData("Mode", assistStatus);
        telemetry.addData("Drive", !Config.FIELD_CENTRIC ? "ROBOT-CENTRIC"
                : (pinpoint == null ? "FIELD-CENTRIC (NO PINPOINT - acting robot-centric)" : "FIELD-CENTRIC"));
        telemetry.addData("Heading vs Zero", "%.1f°", normalizeDegrees(robotHeadingDeg - fieldHeadingOffsetDeg));
        telemetry.addData("Alliance / Battery", "%s | %.2f V", lockedRed ? "RED" : "BLUE", volts);
        telemetry.addData("Pose", "X %5.1f | Y %5.1f | H %5.1f°", robotX, robotY, robotHeadingDeg);
        telemetry.addData("Tag", tag != null
                ? String.format(Locale.US, "#%d (%d allowed in view) | forward %.1f in, right %.1f in",
                tag.tagId, tag.tagCount, tag.forwardIn, tag.rightIn)
                : "None");
        telemetry.addData("Tag raw (x, y, z in)", tag != null
                ? String.format(Locale.US, "%.1f, %.1f, %.1f", tag.rawX, tag.rawY, tag.rawZ)
                : "None");
        telemetry.addData("Tag Debug", tagDebug);
        telemetry.addData("Align Debug", alignDebug);
        telemetry.addData("Balls", ball != null
                ? String.format(Locale.US, "%d in biggest group, tx %+.1f°, height %.2f",
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

                        Mat roiDist = dist.submat(rect);
                        Mat roiMask = Mat.zeros(rect.size(), CvType.CV_8UC1);
                        Imgproc.drawContours(roiMask, Collections.singletonList(contour), 0, new Scalar(255), -1,
                                Imgproc.LINE_8, noHierarchy, 0, new org.opencv.core.Point(-rect.x, -rect.y));
                        Core.MinMaxLocResult mm = Core.minMaxLoc(roiDist, roiMask);
                        roiDist.release();
                        roiMask.release();

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
                Collections.sort(members, (p1, p2) -> Double.compare(p2.bottom, p1.bottom));
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
                    Imgproc.rectangle(input,
                            new org.opencv.core.Point(b.left / scale, b.top / scale),
                            new org.opencv.core.Point(b.right / scale, b.bottom / scale),
                            inBest ? new Scalar(0, 255, 0) : new Scalar(255, 255, 0), 2);
                }
                Imgproc.circle(input, new org.opencv.core.Point(centerX / scale, (bottom[best] - 10) / scale),
                        10, new Scalar(255, 0, 0), 3);
                Imgproc.putText(input, String.valueOf(groupCount[best]),
                        new org.opencv.core.Point(centerX / scale + 14, (bottom[best] - 10) / scale),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.9, new Scalar(255, 0, 0), 2);
            } catch (Exception e) {
                latestBalls = null;
            }
            return input;
        }
    }
}
