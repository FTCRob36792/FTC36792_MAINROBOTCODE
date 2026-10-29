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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * BIOBUZZ TeleOp starter:
 * - Field-centric mecanum drive, using the Pinpoint heading.
 * - Continuously reads Pinpoint X/Y/heading.
 * - Tracks the BIOBUZZ AprilTag cell groups and last estimated field locations.
 * - Limelight 3 runs its AprilTag pipeline continuously for shooting-target tracking.
 * - Logitech webcam runs an independent EasyOpenCV HSV color-blob pipeline concurrently.
 * - A: start/stop AprilTag-based alignment to the currently observed open cell for this alliance.
 * - B: start/stop approaching the largest visible webcam blob.
 * - Y / Options: reset field-centric drive orientation to current robot heading.
 * - Hold Right Bumper (RB) + D-Pad UP/RIGHT/DOWN/LEFT: spin individual wheels forward to test motor directions.
 * - FTC Dashboard Live Tuning (@Config): tune motor reversals, Pinpoint offsets, and webcam exposure live!
 */
@TeleOp(name = "BIOBUZZ Field-Centric Vision TeleOp", group = "BIOBUZZ")
public class BioBuzzFieldCentricTeleOp extends LinearOpMode {

    // =====================================================================
    // ALL ROBOT-SPECIFIC CONFIGURATION / TUNING PARAMETERS LIVE HERE.
    // Annotated with @Config so all fields are live-editable on FTC Dashboard!
    // =====================================================================
    @com.acmerobotics.dashboard.config.Config
    public static class Config {
        // =====================================================================
        // 1. HARDWARE CONFIGURATION NAMES
        // Must match your Driver Station Robot Configuration names exactly.
        // =====================================================================
        public static String FRONT_LEFT_NAME = "front_left";
        public static String FRONT_RIGHT_NAME = "front_right";
        public static String BACK_LEFT_NAME = "back_left";
        public static String BACK_RIGHT_NAME = "back_right";
        public static String PINPOINT_NAME = "pinpoint";
        public static String LIMELIGHT_NAME = "limelight";
        public static String WEBCAM_NAME = "Webcam 1";

        // =====================================================================
        // 2. MOTOR DIRECTION REVERSALS
        // Live-tunable on FTC Dashboard. Use Gamepad 1 [RB + D-Pad] wheel test mode:
        // If a wheel spins backward during test, flip its boolean to true.
        // =====================================================================
        public static boolean REVERSE_FRONT_LEFT = false;  // Front Left Motor Direction
        public static boolean REVERSE_FRONT_RIGHT = true;  // Front Right Motor Direction
        public static boolean REVERSE_BACK_LEFT = false;   // Back Left Motor Direction
        public static boolean REVERSE_BACK_RIGHT = true;   // Back Right Motor Direction

        // =====================================================================
        // 3. GO BILDA PINPOINT ODOMETRY CONFIGURATION
        // Distance from tracking center to pods in millimeters (Positive or Negative).
        // Appropriate values: Measure actual pod distances with calipers/ruler.
        // =====================================================================
        public static double PINPOINT_X_POD_OFFSET_MM = 0.0; // Range: -200.0 to +200.0 mm
        public static double PINPOINT_Y_POD_OFFSET_MM = 0.0; // Range: -200.0 to +200.0 mm
        public static boolean REVERSE_X_ENCODER = false;      // True if forward motion decreases X
        public static boolean REVERSE_Y_ENCODER = false;      // True if right strafe decreases Y
        public static GoBildaPinpointDriver.GoBildaOdometryPods ODOMETRY_POD_TYPE =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        // =====================================================================
        // 4. LOGITECH WEBCAM EXPOSURE & GAIN CONTROL
        // Tune on FTC Dashboard to prevent image blur or washed out colors.
        // =====================================================================
        public static boolean WEBCAM_MANUAL_EXPOSURE = false; // False = Continuous Auto Exposure; True = Fixed Manual
        public static long WEBCAM_EXPOSURE_MS = 15;           // Appropriate values: 5ms to 30ms (lower = less motion blur)
        public static int WEBCAM_GAIN = 100;                  // Appropriate values: 0 to 255 (higher = brighter image)

        // =====================================================================
        // 5. MATCH ALLIANCE & SAFETY ASSIST TOGGLES
        // =====================================================================
        public static boolean RED_ALLIANCE = true;                // True = RED Alliance (Tags 30-37); False = BLUE (Tags 38-45)
        public static boolean ENABLE_APRILTAG_AUTO_ALIGN = false; // Safety lock for Gamepad 1 [A] AprilTag Assist
        public static boolean ENABLE_BLOB_AUTO_APPROACH = false;  // Safety lock for Gamepad 1 [B] Color Blob Assist

        // =====================================================================
        // 6. ROBOT STARTING FIELD POSE
        // Set match starting position in field inches and heading.
        // =====================================================================
        public static double START_X_IN = 0.0;        // Initial X field position (Inches)
        public static double START_Y_IN = 0.0;        // Initial Y field position (Inches)
        public static double START_HEADING_DEG = 0.0; // Initial heading (Degrees: 0° = Facing Audience/Target)

        // =====================================================================
        // 7. DRIVER STICK & SPEED LIMIT CAPS
        // Prevents small joystick drift and limits speed during automatic assists.
        // =====================================================================
        public static double STICK_DEADZONE = 0.16;       // Appropriate values: 0.08 to 0.20 (16% threshold)
        public static double MAX_AUTO_TRANSLATION = 0.20; // Max strafe/forward power cap in assist (0.15 to 0.40)
        public static double MAX_AUTO_TURN = 0.12;        // Max turning power cap in assist (0.08 to 0.25)

        // =====================================================================
        // 8. APRILTAG ALIGNMENT CONTROLLER (Gamepad 1 A)
        // Proportional (Kp) drive controller for auto-aligning to scoring cells.
        // =====================================================================
        public static double TARGET_STANDOFF_IN = 36.0;   // Distance in inches from cell tag to stop at (12.0 to 48.0 in)
        public static double POSITION_KP = 0.025;         // Drive power per inch of error (Appropriate: 0.015 to 0.040)
        public static double HEADING_KP = 0.012;          // Turn power per degree of error (Appropriate: 0.008 to 0.020)
        public static double POSITION_TOLERANCE_IN = 2.5; // Stop threshold for position error (Appropriate: 1.0 to 3.5 in)
        public static double HEADING_TOLERANCE_DEG = 3.0; // Stop threshold for heading error (Appropriate: 1.5 to 5.0 deg)

        public static double CAMERA_TAG_ROLL_OFFSET_DEG = 0.0;
        public static double CELL_UPRIGHT_ROLL_LIMIT_DEG = 90.0;
        public static double MIN_DISTANCE_FOR_BEARING_IN = 4.0;
        public static double CELL_POSITION_SMOOTHING_ALPHA = 0.25;

        // =====================================================================
        // 9. LOGITECH WEBCAM OPENCV HSV COLOR BLOB TRACKING (Gamepad 1 B)
        // HSV Color thresholds for target sample/specimen detection.
        // OpenCV Hue: 0-179 | Saturation: 0-255 | Value/Brightness: 0-255
        // =====================================================================
        public static int HSV_H_MIN = 20;                     // Yellow Hue Min (Appropriate: 15 to 25)
        public static int HSV_H_MAX = 38;                     // Yellow Hue Max (Appropriate: 35 to 45)
        public static int HSV_S_MIN = 80;                     // Saturation Min (Appropriate: 60 to 120, ignores grey/white)
        public static int HSV_S_MAX = 255;                    // Saturation Max
        public static int HSV_V_MIN = 60;                     // Value/Brightness Min (Appropriate: 40 to 100, ignores shadows)
        public static int HSV_V_MAX = 255;                    // Value/Brightness Max
        public static double MIN_BLOB_AREA_PX = 350.0;        // Min pixel area to reject noise (Appropriate: 200 to 800 px)
        public static double BLOB_STOP_AREA_PERCENT = 4.0;    // Target screen area % to stop at (Appropriate: 2.0% to 8.0%)
        public static double BLOB_FORWARD_KP = 0.055;         // Forward power per area % remaining (Appropriate: 0.03 to 0.08)
        public static double BLOB_STRAFE_KP = 0.014;          // Strafe centering power per degree offset (0.008 to 0.025)
        public static double BLOB_CENTER_TOLERANCE_DEG = 2.5; // Centering tolerance in degrees (Appropriate: 1.0 to 4.0 deg)

        // Limelight & AprilTag ID Ranges
        public static int APRILTAG_PIPELINE = 0;
        public static int LIMELIGHT_POLL_RATE_HZ = 100;
        public static long MAX_RESULT_AGE_MS = 250;
        public static final int[] CELL_TAG_ID_MIN = {30, 34, 38, 42};
        public static final int[] CELL_TAG_ID_MAX = {33, 37, 41, 45};
        public static int WEBCAM_WIDTH = 640;
        public static int WEBCAM_HEIGHT = 480;
        public static OpenCvCameraRotation WEBCAM_ROTATION = OpenCvCameraRotation.UPRIGHT;
        public static double WEBCAM_HORIZONTAL_FOV_DEG = 60.0;

        public static long TARGET_STATE_TIMEOUT_MS = 1500;
        public static long VISION_LOSS_TIMEOUT_MS = 700;
        public static int TELEMETRY_INTERVAL_MS = 50;
    }

    // ------------------------------ Hardware ------------------------------
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private OpenCvWebcam webcam;
    private final WebcamBlobPipeline webcamBlobPipeline = new WebcamBlobPipeline();
    private volatile BlobTarget latestWebcamBlob;
    private volatile long latestWebcamBlobMs = 0;
    private volatile boolean webcamReady = false;
    private volatile String webcamStatus = "NOT INITIALIZED";

    // Cell indices: 0 red far side, 1 red audience side, 2 blue audience side, 3 blue far side.
    private final CellTrack[] cells = {
            new CellTrack("Red far-side cell", 0),
            new CellTrack("Red audience-side cell", 1),
            new CellTrack("Blue audience-side cell", 2),
            new CellTrack("Blue far-side cell", 3)
    };
    private final HiveTrack[] hives = {
            new HiveTrack(0, 1), // red HIVE has two cells
            new HiveTrack(2, 3)  // blue HIVE has two cells
    };

    private boolean autoAlignActive = false;
    private boolean blobApproachActive = false;
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousY = false;
    private boolean previousOptions = false;

    private double fieldHeadingOffsetDeg = 0.0;

    private double lastXOffset = Double.NaN;
    private double lastYOffset = Double.NaN;
    private boolean lastXEncoderRev = false;
    private boolean lastYEncoderRev = false;

    private boolean lastManualExposure = false;
    private long lastExposureMs = -1;
    private int lastGain = -1;

    private long lastAlignTagSeenMs = 0;
    private long lastBlobSeenMs = 0;
    private boolean alignGoalCaptured = false;
    private int alignCellIndex = -1;
    private double alignGoalXIn;
    private double alignGoalYIn;
    private double alignGoalHeadingDeg;
    private String assistStatus = "MANUAL DRIVE";

    @Override
    public void runOpMode() {
        initializeHardware();
        configurePinpoint();

        limelight.setPollRateHz(Config.LIMELIGHT_POLL_RATE_HZ);
        limelight.start();
        limelight.pipelineSwitch(Config.APRILTAG_PIPELINE);
        initializeWebcam();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(Config.TELEMETRY_INTERVAL_MS);

        fieldHeadingOffsetDeg = Config.START_HEADING_DEG;

        boolean previousInitX = false;

        while (!isStarted() && !isStopRequested()) {
            boolean pressedX = gamepad1.x && !previousInitX;
            if (pressedX) {
                Config.RED_ALLIANCE = !Config.RED_ALLIANCE;
            }
            previousInitX = gamepad1.x;

            updatePinpointConfigIfNeeded();
            updateWebcamExposureIfNeeded();
            pinpoint.update();
            updateInitTelemetry();
            sleep(20);
        }

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            updatePinpointConfigIfNeeded();
            updateWebcamExposureIfNeeded();

            // Pinpoint must be updated once per loop before reading its new values.
            pinpoint.update();
            double robotX = pinpoint.getPosX(DistanceUnit.INCH);
            double robotY = pinpoint.getPosY(DistanceUnit.INCH);
            double robotHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

            handleButtons(now);

            boolean manualOverride = hasManualStickInput();
            if (manualOverride && (autoAlignActive || blobApproachActive)) {
                cancelAssist("MANUAL OVERRIDE");
            }

            TagChoice bestAllianceTag = null;
            BlobTarget blobTarget = (latestWebcamBlob != null
                    && now - latestWebcamBlobMs <= Config.VISION_LOSS_TIMEOUT_MS)
                    ? latestWebcamBlob : null;
            LLResult result = limelight.getLatestResult();
            boolean freshResult = result != null
                    && result.isValid()
                    && result.getPipelineIndex() == Config.APRILTAG_PIPELINE
                    && limelight.getTimeSinceLastUpdate() <= Config.MAX_RESULT_AGE_MS;

            if (freshResult) {
                bestAllianceTag = processAprilTags(result, now, robotX, robotY, robotHeadingDeg);
            }

            if (autoAlignActive) {
                runAprilTagAlign(bestAllianceTag, now, robotX, robotY, robotHeadingDeg);
            } else if (blobApproachActive) {
                runBlobApproach(blobTarget, now);
            } else {
                runFieldCentricDrive(robotHeadingDeg);
                if (!assistStatus.startsWith("MOTOR TEST")) {
                    assistStatus = manualOverride ? "MANUAL DRIVE" : "READY";
                }
            }

            updateTelemetry(now, robotX, robotY, robotHeadingDeg, bestAllianceTag, blobTarget);
            idle();
        }

        stopDrive();
        limelight.stop();
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, Config.FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, Config.FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, Config.BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, Config.BACK_RIGHT_NAME);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Config.PINPOINT_NAME);
        limelight = hardwareMap.get(Limelight3A.class, Config.LIMELIGHT_NAME);
    }

    private void initializeWebcam() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, Config.WEBCAM_NAME);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(webcamBlobPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Config.WEBCAM_WIDTH, Config.WEBCAM_HEIGHT, Config.WEBCAM_ROTATION);
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
                webcamReady = true;
                webcamStatus = "STREAMING " + Config.WEBCAM_WIDTH + "x" + Config.WEBCAM_HEIGHT;
            }

            @Override
            public void onError(int errorCode) {
                webcamReady = false;
                webcamStatus = "CAMERA ERROR " + errorCode;
            }
        });
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(Config.PINPOINT_X_POD_OFFSET_MM, Config.PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(Config.ODOMETRY_POD_TYPE);
        pinpoint.setEncoderDirections(
                Config.REVERSE_X_ENCODER ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Config.REVERSE_Y_ENCODER ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                Config.START_X_IN,
                Config.START_Y_IN,
                AngleUnit.DEGREES,
                Config.START_HEADING_DEG
        ));

        lastXOffset = Config.PINPOINT_X_POD_OFFSET_MM;
        lastYOffset = Config.PINPOINT_Y_POD_OFFSET_MM;
        lastXEncoderRev = Config.REVERSE_X_ENCODER;
        lastYEncoderRev = Config.REVERSE_Y_ENCODER;
    }

    private void updatePinpointConfigIfNeeded() {
        if (pinpoint == null) return;
        if (Config.PINPOINT_X_POD_OFFSET_MM != lastXOffset
                || Config.PINPOINT_Y_POD_OFFSET_MM != lastYOffset
                || Config.REVERSE_X_ENCODER != lastXEncoderRev
                || Config.REVERSE_Y_ENCODER != lastYEncoderRev) {

            pinpoint.setOffsets(Config.PINPOINT_X_POD_OFFSET_MM, Config.PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderDirections(
                    Config.REVERSE_X_ENCODER ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    Config.REVERSE_Y_ENCODER ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            lastXOffset = Config.PINPOINT_X_POD_OFFSET_MM;
            lastYOffset = Config.PINPOINT_Y_POD_OFFSET_MM;
            lastXEncoderRev = Config.REVERSE_X_ENCODER;
            lastYEncoderRev = Config.REVERSE_Y_ENCODER;
        }
    }

    private void updateWebcamExposureIfNeeded() {
        if (webcam == null) return;

        try {
            ExposureControl exposureControl = webcam.getExposureControl();
            GainControl gainControl = webcam.getGainControl();

            if (exposureControl != null && (Config.WEBCAM_MANUAL_EXPOSURE != lastManualExposure
                    || Config.WEBCAM_EXPOSURE_MS != lastExposureMs
                    || Config.WEBCAM_GAIN != lastGain)) {

                if (Config.WEBCAM_MANUAL_EXPOSURE) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    exposureControl.setExposure(Config.WEBCAM_EXPOSURE_MS, TimeUnit.MILLISECONDS);
                    if (gainControl != null) {
                        int clampedGain = Math.min(gainControl.getMaxGain(), Math.max(gainControl.getMinGain(), Config.WEBCAM_GAIN));
                        gainControl.setGain(clampedGain);
                    }
                } else {
                    exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
                }

                lastManualExposure = Config.WEBCAM_MANUAL_EXPOSURE;
                lastExposureMs = Config.WEBCAM_EXPOSURE_MS;
                lastGain = Config.WEBCAM_GAIN;
            }
        } catch (Exception ignored) {
            // Camera control unsupported on some devices
        }
    }

    private void handleButtons(long now) {
        boolean pressedA = gamepad1.a && !previousA;
        boolean pressedB = gamepad1.b && !previousB;
        boolean pressedY = gamepad1.y && !previousY;
        boolean pressedOptions = (gamepad1.options || gamepad1.back) && !previousOptions;

        if (pressedA) {
            if (autoAlignActive) {
                cancelAssist("ALIGN CANCELLED");
            } else if (!Config.ENABLE_APRILTAG_AUTO_ALIGN) {
                assistStatus = "ALIGN DISABLED: set Config.ENABLE_APRILTAG_AUTO_ALIGN after calibration";
            } else {
                blobApproachActive = false;
                autoAlignActive = true;
                alignGoalCaptured = false;
                alignCellIndex = -1;
                lastAlignTagSeenMs = now;
                assistStatus = "WAITING FOR OPEN ALLIANCE CELL TAG";
            }
        }

        if (pressedB) {
            if (blobApproachActive) {
                cancelAssist("BLOB APPROACH CANCELLED");
            } else if (!Config.ENABLE_BLOB_AUTO_APPROACH) {
                assistStatus = "BLOB APPROACH DISABLED: set Config.ENABLE_BLOB_AUTO_APPROACH after calibration";
            } else {
                autoAlignActive = false;
                alignGoalCaptured = false;
                blobApproachActive = true;
                lastBlobSeenMs = now;
                assistStatus = "WAITING FOR LOGITECH BLOB";
            }
        }

        // Reset field-centric drive alignment to current robot heading when Y or Options/Back is pressed
        if (pressedY || pressedOptions) {
            fieldHeadingOffsetDeg = pinpoint.getHeading(AngleUnit.DEGREES);
            assistStatus = "FIELD HEADING RESET TO CURRENT ORIENTATION";
        }

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousY = gamepad1.y;
        previousOptions = gamepad1.options || gamepad1.back;
    }

    private boolean hasManualStickInput() {
        return Math.abs(gamepad1.left_stick_x) > Config.STICK_DEADZONE
                || Math.abs(gamepad1.left_stick_y) > Config.STICK_DEADZONE
                || Math.abs(gamepad1.right_stick_x) > Config.STICK_DEADZONE;
    }

    private void cancelAssist(String reason) {
        autoAlignActive = false;
        blobApproachActive = false;
        alignGoalCaptured = false;
        alignCellIndex = -1;
        assistStatus = reason;
        stopDrive();
    }

    // ------------------------------ Tag logic -----------------------------

    private TagChoice processAprilTags(LLResult result,
                                       long now,
                                       double robotX,
                                       double robotY,
                                       double robotHeadingDeg) {
        CellObservation[] observations = new CellObservation[cells.length];
        TagChoice[] bestTagForCell = new TagChoice[cells.length];
        for (int i = 0; i < cells.length; i++) {
            observations[i] = new CellObservation();
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult tag : fiducials) {
                int id = tag.getFiducialId();
                int cellIndex = cellIndexForTag(id);
                if (cellIndex < 0) {
                    continue; // Ignore tags outside the BIOBUZZ cell clusters.
                }

                CellObservation observation = observations[cellIndex];
                observation.tagCount++;
                observation.areaSum += tag.getTargetArea();

                try {
                    Pose3D cameraTargetPose = tag.getTargetPoseCameraSpace();
                    if (cameraTargetPose != null && cameraTargetPose.getOrientation() != null) {
                        double roll = normalizeDegrees(
                                cameraTargetPose.getOrientation().getRoll(AngleUnit.DEGREES)
                                        - Config.CAMERA_TAG_ROLL_OFFSET_DEG);
                        if (isFinite(roll)) {
                            observation.rollSum += roll;
                            observation.rollSamples++;
                            if (Math.abs(roll) < Config.CELL_UPRIGHT_ROLL_LIMIT_DEG) {
                                observation.uprightVotes++;
                            } else {
                                observation.invertedVotes++;
                            }
                        }
                    }
                } catch (RuntimeException ignored) {
                }

                TagChoice oldChoice = bestTagForCell[cellIndex];
                if (oldChoice == null || tag.getTargetArea() > oldChoice.area) {
                    bestTagForCell[cellIndex] = new TagChoice(tag, cellIndex, tag.getTargetArea());
                }

                try {
                    Pose3D targetPose = tag.getTargetPoseRobotSpace();
                    if (targetPose != null && targetPose.getPosition() != null) {
                        Position relative = targetPose.getPosition().toUnit(DistanceUnit.INCH);
                        double targetForward = relative.x;
                        double targetRight = relative.y;
                        double headingRad = Math.toRadians(robotHeadingDeg);

                        double targetFieldX = robotX
                                + targetForward * Math.cos(headingRad)
                                + targetRight * Math.sin(headingRad);
                        double targetFieldY = robotY
                                + targetForward * Math.sin(headingRad)
                                - targetRight * Math.cos(headingRad);

                        if (isFinite(targetFieldX) && isFinite(targetFieldY)) {
                            observation.positionXSum += targetFieldX;
                            observation.positionYSum += targetFieldY;
                            observation.positionSamples++;
                        }
                    }
                } catch (RuntimeException ignored) {
                }
            }
        }

        for (int i = 0; i < cells.length; i++) {
            CellObservation observation = observations[i];
            if (observation.tagCount > 0) {
                cells[i].lastTagSeenMs = now;
            }
            if (observation.positionSamples > 0) {
                double meanX = observation.positionXSum / observation.positionSamples;
                double meanY = observation.positionYSum / observation.positionSamples;
                cells[i].updateFieldEstimate(meanX, meanY, now);
            }
            if (observation.rollSamples > 0) {
                cells[i].updateRoll(observation.rollSum / observation.rollSamples, now);
            }
        }

        for (HiveTrack hive : hives) {
            int bestCandidate = -1;
            double bestConfidence = -1.0;
            int[] pair = {hive.cellA, hive.cellB};
            for (int observedCellIndex : pair) {
                CellObservation observation = observations[observedCellIndex];
                if (observation.rollSamples == 0
                        || observation.uprightVotes == observation.invertedVotes) {
                    continue;
                }

                boolean tagClusterUpright = observation.uprightVotes > observation.invertedVotes;
                int candidateOpenCell = tagClusterUpright
                        ? observedCellIndex
                        : (observedCellIndex == hive.cellA ? hive.cellB : hive.cellA);
                double voteMargin = Math.abs(observation.uprightVotes - observation.invertedVotes)
                        / (double) observation.rollSamples;
                double confidence = observation.areaSum * voteMargin;
                if (confidence > bestConfidence) {
                    bestConfidence = confidence;
                    bestCandidate = candidateOpenCell;
                }
            }

            if (bestCandidate >= 0) {
                hive.openCellIndex = bestCandidate;
                hive.lastObservedMs = now;
            } else if (hive.lastObservedMs > 0
                    && now - hive.lastObservedMs > Config.TARGET_STATE_TIMEOUT_MS) {
                hive.openCellIndex = -1;
            }
        }

        int ourHiveIndex = Config.RED_ALLIANCE ? 0 : 1;
        int openCell = hives[ourHiveIndex].openCellIndex;
        if (openCell >= 0 && bestTagForCell[openCell] != null) {
            return bestTagForCell[openCell];
        }
        return null;
    }

    private int cellIndexForTag(int id) {
        for (int i = 0; i < Config.CELL_TAG_ID_MIN.length; i++) {
            if (id >= Config.CELL_TAG_ID_MIN[i] && id <= Config.CELL_TAG_ID_MAX[i]) {
                return i;
            }
        }
        return -1;
    }

    private String getCellState(int cellIndex, long now) {
        HiveTrack hive = hives[cellIndex < 2 ? 0 : 1];
        if (hive.lastObservedMs == 0 || now - hive.lastObservedMs > Config.TARGET_STATE_TIMEOUT_MS
                || hive.openCellIndex < 0) {
            return "UNKNOWN";
        }
        return hive.openCellIndex == cellIndex ? "OPEN" : "CLOSED";
    }

    // ---------------------------- Blob approach ---------------------------

    private void runBlobApproach(BlobTarget target, long now) {
        if (target == null) {
            stopDrive();
            assistStatus = "BLOB NOT VISIBLE - STOPPED";
            if (now - lastBlobSeenMs > Config.VISION_LOSS_TIMEOUT_MS) {
                cancelAssist("BLOB LOST - ASSIST CANCELLED");
            }
            return;
        }

        lastBlobSeenMs = now;
        double strafeRight = clip(target.txDegrees * Config.BLOB_STRAFE_KP, -Config.MAX_AUTO_TRANSLATION, Config.MAX_AUTO_TRANSLATION);
        double forward = clip((Config.BLOB_STOP_AREA_PERCENT - target.areaPercent) * Config.BLOB_FORWARD_KP,
                0.0, Config.MAX_AUTO_TRANSLATION);

        if (target.areaPercent >= Config.BLOB_STOP_AREA_PERCENT
                && Math.abs(target.txDegrees) <= Config.BLOB_CENTER_TOLERANCE_DEG) {
            blobApproachActive = false;
            stopDrive();
            assistStatus = "GAME ELEMENT CENTERED / APPROACH STOPPED";
            return;
        }

        setRobotDrive(forward, strafeRight, 0.0);
        assistStatus = String.format(Locale.US, "BLOB APPROACH: tx %.1f deg, area %.2f%%",
                target.txDegrees, target.areaPercent);
    }

    // ----------------------------- Tag align ------------------------------

    private void runAprilTagAlign(TagChoice visibleTag,
                                  long now,
                                  double robotX,
                                  double robotY,
                                  double robotHeadingDeg) {
        if (visibleTag == null) {
            stopDrive();
            assistStatus = "TAG LOST - STOPPED, SEARCHING";
            if (now - lastAlignTagSeenMs > Config.VISION_LOSS_TIMEOUT_MS) {
                cancelAssist("TAG LOST - ALIGN CANCELLED");
            }
            return;
        }

        lastAlignTagSeenMs = now;
        if (alignGoalCaptured && visibleTag.cellIndex != alignCellIndex) {
            cancelAssist("OPEN CELL CHANGED - ALIGN CANCELLED");
            return;
        }

        if (!alignGoalCaptured) {
            CellTrack targetCell = cells[visibleTag.cellIndex];
            if (!targetCell.hasFieldEstimate) {
                stopDrive();
                assistStatus = "WAITING FOR 3D TARGET LOCATION";
                return;
            }

            double targetX = targetCell.fieldXIn;
            double targetY = targetCell.fieldYIn;
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double distance = Math.hypot(dx, dy);
            double bearingRad = distance > Config.MIN_DISTANCE_FOR_BEARING_IN
                    ? Math.atan2(dy, dx)
                    : Math.toRadians(robotHeadingDeg);

            alignGoalXIn = targetX - Config.TARGET_STANDOFF_IN * Math.cos(bearingRad);
            alignGoalYIn = targetY - Config.TARGET_STANDOFF_IN * Math.sin(bearingRad);
            alignGoalHeadingDeg = normalizeDegrees(Math.toDegrees(bearingRad));
            alignCellIndex = visibleTag.cellIndex;
            alignGoalCaptured = true;
        }

        double errorX = alignGoalXIn - robotX;
        double errorY = alignGoalYIn - robotY;
        double distanceError = Math.hypot(errorX, errorY);
        double headingError = normalizeDegrees(alignGoalHeadingDeg - robotHeadingDeg);

        if (distanceError <= Config.POSITION_TOLERANCE_IN
                && Math.abs(headingError) <= Config.HEADING_TOLERANCE_DEG) {
            autoAlignActive = false;
            alignGoalCaptured = false;
            stopDrive();
            assistStatus = "ALIGNMENT COMPLETE";
            return;
        }

        double fieldXCommand = clip(errorX * Config.POSITION_KP,
                -Config.MAX_AUTO_TRANSLATION, Config.MAX_AUTO_TRANSLATION);
        double fieldYCommand = clip(errorY * Config.POSITION_KP,
                -Config.MAX_AUTO_TRANSLATION, Config.MAX_AUTO_TRANSLATION);
        double commandMagnitude = Math.hypot(fieldXCommand, fieldYCommand);
        if (commandMagnitude > Config.MAX_AUTO_TRANSLATION) {
            fieldXCommand *= Config.MAX_AUTO_TRANSLATION / commandMagnitude;
            fieldYCommand *= Config.MAX_AUTO_TRANSLATION / commandMagnitude;
        }

        double headingRad = Math.toRadians(robotHeadingDeg);
        double forward = fieldXCommand * Math.cos(headingRad) + fieldYCommand * Math.sin(headingRad);
        double strafeRight = fieldXCommand * Math.sin(headingRad) - fieldYCommand * Math.cos(headingRad);
        double turn = clip(-headingError * Config.HEADING_KP, -Config.MAX_AUTO_TURN, Config.MAX_AUTO_TURN);

        setRobotDrive(forward, strafeRight, turn);
        assistStatus = String.format(Locale.US,
                "ALIGN %s: %.1f in, heading error %.1f deg",
                cells[alignCellIndex].name, distanceError, headingError);
    }

    // ------------------------------- Driving ------------------------------

    private void runFieldCentricDrive(double robotHeadingDeg) {
        // Individual Wheel Diagnostic Test Mode:
        // Hold Right Bumper (RB) + D-Pad directions to spin individual wheels forward
        if (gamepad1.right_bumper) {
            double testPower = 0.35;
            double fl = gamepad1.dpad_up ? testPower : 0.0;
            double fr = gamepad1.dpad_right ? testPower : 0.0;
            double bl = gamepad1.dpad_down ? testPower : 0.0;
            double br = gamepad1.dpad_left ? testPower : 0.0;

            if (fl != 0 || fr != 0 || bl != 0 || br != 0) {
                if (Config.REVERSE_FRONT_LEFT) fl = -fl;
                if (Config.REVERSE_FRONT_RIGHT) fr = -fr;
                if (Config.REVERSE_BACK_LEFT) bl = -bl;
                if (Config.REVERSE_BACK_RIGHT) br = -br;

                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);

                if (gamepad1.dpad_up) assistStatus = "MOTOR TEST: FRONT LEFT";
                else if (gamepad1.dpad_right) assistStatus = "MOTOR TEST: FRONT RIGHT";
                else if (gamepad1.dpad_down) assistStatus = "MOTOR TEST: BACK LEFT";
                else if (gamepad1.dpad_left) assistStatus = "MOTOR TEST: BACK RIGHT";

                return;
            }
        }

        double fieldRight = gamepad1.left_stick_x;
        double fieldForward = -gamepad1.left_stick_y;
        double relativeHeading = Math.toRadians(normalizeDegrees(robotHeadingDeg - fieldHeadingOffsetDeg));

        double strafeRight = fieldRight * Math.cos(relativeHeading)
                + fieldForward * Math.sin(relativeHeading);
        double forward = -fieldRight * Math.sin(relativeHeading)
                + fieldForward * Math.cos(relativeHeading);
        double turn = gamepad1.right_stick_x;

        setRobotDrive(forward, strafeRight, turn);
    }

    private void setRobotDrive(double forward, double strafeRight, double turn) {
        double fl = forward + strafeRight + turn;
        double fr = forward - strafeRight - turn;
        double bl = forward - strafeRight + turn;
        double br = forward + strafeRight - turn;

        // Apply live motor direction toggles from Config / FTC Dashboard
        if (Config.REVERSE_FRONT_LEFT) fl = -fl;
        if (Config.REVERSE_FRONT_RIGHT) fr = -fr;
        if (Config.REVERSE_BACK_LEFT) bl = -bl;
        if (Config.REVERSE_BACK_RIGHT) br = -br;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }

    // ------------------------------ Telemetry -----------------------------

    private void updateInitTelemetry() {
        telemetry.addLine("=== BIOBUZZ TELEOP INITIALIZATION ===");
        telemetry.addData("Alliance Target", Config.RED_ALLIANCE ? "RED Cells (IDs 30-37)" : "BLUE Cells (IDs 38-45)");
        telemetry.addData("Start Pose", "X: %.1f in | Y: %.1f in | H: %.1f°",
                Config.START_X_IN, Config.START_Y_IN, Config.START_HEADING_DEG);
        telemetry.addData("Auto Align", Config.ENABLE_APRILTAG_AUTO_ALIGN ? "ENABLED" : "DISABLED (Config toggle)");
        telemetry.addData("Blob Approach", Config.ENABLE_BLOB_AUTO_APPROACH ? "ENABLED" : "DISABLED (Config toggle)");

        telemetry.addLine("\n--- SENSOR STATUS ---");
        telemetry.addData("Limelight 3A", limelight.isConnected()
                ? String.format(Locale.US, "CONNECTED (Pipe %d)", Config.APRILTAG_PIPELINE)
                : "DISCONNECTED");
        telemetry.addData("Logitech Webcam", webcamStatus);
        telemetry.addData("Pinpoint Odometry", "X: %.1f in | Y: %.1f in | H: %.1f°",
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("\n--- CONTROLS ---");
        telemetry.addLine("  [X Button] Toggle Alliance Color (Press during INIT)");
        telemetry.addLine("  [A] AprilTag Auto Align | [B] Blob Approach");
        telemetry.addLine("  [Y / Options] Reset Field Driving Heading");
        telemetry.addLine("  [RB + D-Pad] Wheel Direction Test Mode");
        telemetry.addLine("  [Sticks] Manual Field-Centric Drive / Cancel Assist");
        telemetry.update();
    }

    private void updateTelemetry(long now,
                                 double robotX,
                                 double robotY,
                                 double robotHeadingDeg,
                                 TagChoice bestAllianceTag,
                                 BlobTarget blobTarget) {
        // --- SECTION 1: STATUS & DRIVE MODE ---
        telemetry.addLine("=== SYSTEM & DRIVE ===");
        telemetry.addData("Drive Mode", assistStatus);
        telemetry.addData("Alliance", Config.RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.addData("Heading Baseline", "%.1f° (Relative H: %.1f°)",
                fieldHeadingOffsetDeg, normalizeDegrees(robotHeadingDeg - fieldHeadingOffsetDeg));

        // --- SECTION 2: ROBOT POSE (PINPOINT ODOMETRY) ---
        telemetry.addLine("\n=== ROBOT POSE ===");
        telemetry.addData("Position", "X: %6.1f in | Y: %6.1f in", robotX, robotY);
        telemetry.addData("Heading", "%6.1f°", robotHeadingDeg);
        telemetry.addData("Pod Offsets", "X: %.1f mm | Y: %.1f mm", Config.PINPOINT_X_POD_OFFSET_MM, Config.PINPOINT_Y_POD_OFFSET_MM);

        // --- SECTION 3: VISION & TARGETS ---
        telemetry.addLine("\n=== VISION & TARGETS ===");
        telemetry.addData("Limelight", limelight.isConnected()
                ? String.format(Locale.US, "CONNECTED (Pipe %d)", Config.APRILTAG_PIPELINE)
                : "DISCONNECTED");
        String exposureInfo = Config.WEBCAM_MANUAL_EXPOSURE
                ? String.format(Locale.US, "MANUAL (%d ms, gain %d)", Config.WEBCAM_EXPOSURE_MS, Config.WEBCAM_GAIN)
                : "AUTO";
        telemetry.addData("Webcam", "%s [Exposure: %s]", webcamStatus, exposureInfo);

        if (bestAllianceTag != null) {
            telemetry.addData("Tag Target", "%s (Tag #%d) [tx: %+.1f°]",
                    cells[bestAllianceTag.cellIndex].name,
                    bestAllianceTag.result.getFiducialId(),
                    bestAllianceTag.result.getTargetXDegrees());
        } else {
            telemetry.addData("Tag Target", "None in view");
        }

        if (blobTarget != null) {
            telemetry.addData("Blob Target", "%s [tx: %+.1f°, area: %.2f%%]",
                    blobTarget.label, blobTarget.txDegrees, blobTarget.areaPercent);
        } else {
            telemetry.addData("Blob Target", "None in view");
        }

        // --- SECTION 4: CELL TRACKING ---
        telemetry.addLine("\n=== CELL TRACKING ===");
        for (int i = 0; i < cells.length; i++) {
            CellTrack cell = cells[i];
            String state = getCellState(i, now);

            String info;
            if (cell.hasFieldEstimate) {
                double ageSec = cell.lastPositionUpdateMs > 0 ? (now - cell.lastPositionUpdateMs) / 1000.0 : 0.0;
                info = String.format(Locale.US, "[%-7s] X: %5.1f, Y: %5.1f  (%.1fs ago)",
                        state, cell.fieldXIn, cell.fieldYIn, ageSec);
            } else {
                info = String.format(Locale.US, "[%-7s] No location estimate", state);
            }

            telemetry.addData(cell.name, info);
        }

        telemetry.update();
    }

    private static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double normalizeDegrees(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private static class CellObservation {
        int tagCount = 0;
        double areaSum = 0.0;
        double positionXSum = 0.0;
        double positionYSum = 0.0;
        int positionSamples = 0;
        double rollSum = 0.0;
        int rollSamples = 0;
        int uprightVotes = 0;
        int invertedVotes = 0;
    }

    private static class CellTrack {
        final String name;
        final int index;
        boolean hasFieldEstimate = false;
        double fieldXIn = 0.0;
        double fieldYIn = 0.0;
        long lastTagSeenMs = 0;
        long lastPositionUpdateMs = 0;
        double lastRollDeg = Double.NaN;
        long lastRollUpdateMs = 0;

        CellTrack(String name, int index) {
            this.name = name;
            this.index = index;
        }

        void updateFieldEstimate(double x, double y, long now) {
            if (!hasFieldEstimate) {
                fieldXIn = x;
                fieldYIn = y;
                hasFieldEstimate = true;
            } else {
                final double alpha = Config.CELL_POSITION_SMOOTHING_ALPHA;
                fieldXIn = alpha * x + (1.0 - alpha) * fieldXIn;
                fieldYIn = alpha * y + (1.0 - alpha) * fieldYIn;
            }
            lastPositionUpdateMs = now;
        }

        void updateRoll(double rollDeg, long now) {
            lastRollDeg = rollDeg;
            lastRollUpdateMs = now;
        }
    }

    private static class HiveTrack {
        final int cellA;
        final int cellB;
        int openCellIndex = -1;
        long lastObservedMs = 0;

        HiveTrack(int cellA, int cellB) {
            this.cellA = cellA;
            this.cellB = cellB;
        }
    }

    private static class TagChoice {
        final LLResultTypes.FiducialResult result;
        final int cellIndex;
        final double area;

        TagChoice(LLResultTypes.FiducialResult result, int cellIndex, double area) {
            this.result = result;
            this.cellIndex = cellIndex;
            this.area = area;
        }
    }

    private class WebcamBlobPipeline extends OpenCvPipeline {
        private final Mat rgb = new Mat();
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final Mat hierarchy = new Mat();
        private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new org.opencv.core.Size(3, 3));
        private final List<MatOfPoint> contours = new java.util.ArrayList<>();
        private final Scalar lower = new Scalar(Config.HSV_H_MIN, Config.HSV_S_MIN, Config.HSV_V_MIN);
        private final Scalar upper = new Scalar(Config.HSV_H_MAX, Config.HSV_S_MAX, Config.HSV_V_MAX);
        private volatile BlobTarget latest = null;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, lower, upper, mask);
            Imgproc.erode(mask, mask, kernel, new org.opencv.core.Point(-1, -1), 1);
            Imgproc.dilate(mask, mask, kernel, new org.opencv.core.Point(-1, -1), 2);

            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE);
            double bestArea = 0.0;
            Rect bestRect = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > bestArea) {
                    bestArea = area;
                    bestRect = Imgproc.boundingRect(contour);
                }
                contour.release();
            }
            contours.clear();

            if (bestRect != null && bestArea >= Config.MIN_BLOB_AREA_PX) {
                double centerX = bestRect.x + bestRect.width / 2.0;
                double imageCenterX = input.cols() / 2.0;
                double normalizedX = (centerX - imageCenterX) / imageCenterX;
                double tx = normalizedX * (Config.WEBCAM_HORIZONTAL_FOV_DEG / 2.0);
                double areaPercent = 100.0 * bestArea / (input.cols() * (double) input.rows());
                latest = new BlobTarget("OpenCV color blob", tx, areaPercent);
                Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new org.opencv.core.Point(centerX,
                        bestRect.y + bestRect.height / 2.0), 4, new Scalar(255, 0, 0), -1);
            } else {
                latest = null;
            }
            latestWebcamBlob = latest;
            latestWebcamBlobMs = System.currentTimeMillis();
            return input;
        }
    }

    private static class BlobTarget {
        final String label;
        final double txDegrees;
        final double areaPercent;

        BlobTarget(String label, double txDegrees, double areaPercent) {
            this.label = label;
            this.txDegrees = txDegrees;
            this.areaPercent = areaPercent;
        }
    }
}
