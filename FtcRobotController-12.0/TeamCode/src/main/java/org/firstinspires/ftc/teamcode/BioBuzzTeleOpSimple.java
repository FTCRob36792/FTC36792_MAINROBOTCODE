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
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * BIOBUZZ Robust & Efficient TeleOp:
 * - Clean, fail-safe field-centric mecanum driving powered by GoBilda Pinpoint.
 * - Battery-efficient execution: reduces GC pauses, optimizes motor power scaling,
 *   and protects against sensor disconnect crashes.
 * - Full feature set retained:
 *   - Field-Centric drive with Y / Options button re-zeroing.
 *   - RB + D-Pad wheel diagnostic test mode.
 *   - Pre-match X button alliance toggle (RED vs BLUE).
 *   - AprilTag target alignment (A button) & OpenCV color blob approach (B button).
 *   - FTC Dashboard @Config live tuning for motor reversals, offsets, and webcam exposure.
 */
@TeleOp(name = "BIOBUZZ TeleOp (Robust & Efficient)", group = "BIOBUZZ")
public class BioBuzzTeleOpSimple extends LinearOpMode {

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
        public static final int[] CELL_TAG_ID_MIN = {30, 34, 38, 42};
        public static final int[] CELL_TAG_ID_MAX = {33, 37, 41, 45};
    }

    // Hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private OpenCvWebcam webcam;
    private VoltageSensor batteryVoltageSensor;

    // Vision Pipeline
    private final WebcamBlobPipeline webcamBlobPipeline = new WebcamBlobPipeline();
    private volatile BlobTarget latestWebcamBlob;
    private volatile long latestWebcamBlobMs = 0;
    private volatile String webcamStatus = "NOT INITIALIZED";

    // State Tracking
    private boolean autoAlignActive = false;
    private boolean blobApproachActive = false;
    private boolean previousA = false, previousB = false, previousY = false, previousOptions = false, previousInitX = false;

    private double fieldHeadingOffsetDeg = 0.0;
    private double lastXOffset = Double.NaN, lastYOffset = Double.NaN;
    private boolean lastXEncoderRev = false, lastYEncoderRev = false;

    private boolean lastManualExposure = false;
    private long lastExposureMs = -1;
    private int lastGain = -1;

    private String assistStatus = "MANUAL DRIVE";
    private boolean alignGoalCaptured = false;
    private double alignGoalXIn, alignGoalYIn, alignGoalHeadingDeg;
    private long lastAlignTagSeenMs = 0, lastBlobSeenMs = 0;

    // Cell tracking structures
    private static class CellTrack {
        final String name;
        boolean hasFieldEstimate = false;
        double fieldXIn = 0.0, fieldYIn = 0.0;
        long lastPositionUpdateMs = 0;

        CellTrack(String name) { this.name = name; }
    }

    private final CellTrack[] cells = {
            new CellTrack("Red Far Cell"), new CellTrack("Red Aud Cell"),
            new CellTrack("Blue Aud Cell"), new CellTrack("Blue Far Cell")
    };

    @Override
    public void runOpMode() {
        initializeHardware();
        configurePinpoint();

        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(Config.APRILTAG_PIPELINE);
        }
        initializeWebcam();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100); // 100ms transmission reduces CPU load

        fieldHeadingOffsetDeg = Config.START_HEADING_DEG;

        // --- PRE-MATCH INIT LOOP ---
        while (!isStarted() && !isStopRequested()) {
            boolean pressedX = gamepad1.x && !previousInitX;
            if (pressedX) {
                Config.RED_ALLIANCE = !Config.RED_ALLIANCE;
            }
            previousInitX = gamepad1.x;

            updatePinpointConfigIfNeeded();
            updateWebcamExposureIfNeeded();
            if (pinpoint != null) pinpoint.update();

            updateInitTelemetry();
            sleep(20);
        }

        // --- ACTIVE MATCH LOOP ---
        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            updatePinpointConfigIfNeeded();
            updateWebcamExposureIfNeeded();

            double robotX = 0.0, robotY = 0.0, robotHeadingDeg = 0.0;
            if (pinpoint != null) {
                pinpoint.update();
                robotX = pinpoint.getPosX(DistanceUnit.INCH);
                robotY = pinpoint.getPosY(DistanceUnit.INCH);
                robotHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
            }

            handleButtons(now);

            boolean manualOverride = hasManualStickInput();
            if (manualOverride && (autoAlignActive || blobApproachActive)) {
                cancelAssist("MANUAL OVERRIDE");
            }

            TagChoice bestAllianceTag = null;
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid() && result.getPipelineIndex() == Config.APRILTAG_PIPELINE
                        && limelight.getTimeSinceLastUpdate() <= 250) {
                    bestAllianceTag = processAprilTags(result, now, robotX, robotY, robotHeadingDeg);
                }
            }

            BlobTarget blobTarget = (latestWebcamBlob != null && now - latestWebcamBlobMs <= 700)
                    ? latestWebcamBlob : null;

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

            updateTelemetry(robotX, robotY, robotHeadingDeg, bestAllianceTag, blobTarget);
            idle();
        }

        // Clean Shutdown
        stopDrive();
        if (limelight != null) limelight.stop();
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    private void initializeHardware() {
        try {
            frontLeft = hardwareMap.get(DcMotor.class, Config.FRONT_LEFT_NAME);
            frontRight = hardwareMap.get(DcMotor.class, Config.FRONT_RIGHT_NAME);
            backLeft = hardwareMap.get(DcMotor.class, Config.BACK_LEFT_NAME);
            backRight = hardwareMap.get(DcMotor.class, Config.BACK_RIGHT_NAME);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Hardware Warning", "Drive motors missing or misnamed");
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
    }

    private void initializeWebcam() {
        try {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, Config.WEBCAM_NAME);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            webcam.setPipeline(webcamBlobPipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    FtcDashboard.getInstance().startCameraStream(webcam, 30);
                    webcamStatus = "STREAMING 640x480";
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
        pinpoint.setOffsets(Config.PINPOINT_X_POD_OFFSET_MM, Config.PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                Config.REVERSE_X_ENCODER ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Config.REVERSE_Y_ENCODER ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Config.START_X_IN, Config.START_Y_IN, AngleUnit.DEGREES, Config.START_HEADING_DEG));

        lastXOffset = Config.PINPOINT_X_POD_OFFSET_MM;
        lastYOffset = Config.PINPOINT_Y_POD_OFFSET_MM;
        lastXEncoderRev = Config.REVERSE_X_ENCODER;
        lastYEncoderRev = Config.REVERSE_Y_ENCODER;
    }

    private void updatePinpointConfigIfNeeded() {
        if (pinpoint == null) return;
        if (Config.PINPOINT_X_POD_OFFSET_MM != lastXOffset || Config.PINPOINT_Y_POD_OFFSET_MM != lastYOffset
                || Config.REVERSE_X_ENCODER != lastXEncoderRev || Config.REVERSE_Y_ENCODER != lastYEncoderRev) {

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
                    || Config.WEBCAM_EXPOSURE_MS != lastExposureMs || Config.WEBCAM_GAIN != lastGain)) {

                if (Config.WEBCAM_MANUAL_EXPOSURE) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    exposureControl.setExposure(Config.WEBCAM_EXPOSURE_MS, TimeUnit.MILLISECONDS);
                    if (gainControl != null) {
                        gainControl.setGain(Math.min(gainControl.getMaxGain(), Math.max(gainControl.getMinGain(), Config.WEBCAM_GAIN)));
                    }
                } else {
                    exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
                }
                lastManualExposure = Config.WEBCAM_MANUAL_EXPOSURE;
                lastExposureMs = Config.WEBCAM_EXPOSURE_MS;
                lastGain = Config.WEBCAM_GAIN;
            }
        } catch (Exception ignored) {}
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
                assistStatus = "ALIGN DISABLED (Config Safety)";
            } else {
                blobApproachActive = false;
                autoAlignActive = true;
                alignGoalCaptured = false;
                lastAlignTagSeenMs = now;
                assistStatus = "WAITING FOR CELL TAG";
            }
        }

        if (pressedB) {
            if (blobApproachActive) {
                cancelAssist("BLOB CANCELLED");
            } else if (!Config.ENABLE_BLOB_AUTO_APPROACH) {
                assistStatus = "BLOB APPROACH DISABLED (Config Safety)";
            } else {
                autoAlignActive = false;
                blobApproachActive = true;
                lastBlobSeenMs = now;
                assistStatus = "WAITING FOR BLOB";
            }
        }

        if (pressedY || pressedOptions) {
            if (pinpoint != null) fieldHeadingOffsetDeg = pinpoint.getHeading(AngleUnit.DEGREES);
            assistStatus = "HEADING RE-ZEROED";
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
        assistStatus = reason;
        stopDrive();
    }

    private void runFieldCentricDrive(double robotHeadingDeg) {
        // Motor Diagnostic Test Mode (Hold RB + D-Pad)
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
                else if (gamepad1.dpad_left) assistStatus = "MOTOR TEST: BACK RIGHT";
                return;
            }
        }

        double fieldRight = gamepad1.left_stick_x;
        double fieldForward = -gamepad1.left_stick_y;

        // Zero-Power Energy Saving Check
        if (Math.abs(fieldRight) <= Config.STICK_DEADZONE && Math.abs(fieldForward) <= Config.STICK_DEADZONE
                && Math.abs(gamepad1.right_stick_x) <= Config.STICK_DEADZONE) {
            stopDrive();
            return;
        }

        double relativeHeading = Math.toRadians(normalizeDegrees(robotHeadingDeg - fieldHeadingOffsetDeg));
        double strafeRight = fieldRight * Math.cos(relativeHeading) + fieldForward * Math.sin(relativeHeading);
        double forward = -fieldRight * Math.sin(relativeHeading) + fieldForward * Math.cos(relativeHeading);
        double turn = gamepad1.right_stick_x;

        setRobotDrive(forward, strafeRight, turn);
    }

    private void setRobotDrive(double forward, double strafeRight, double turn) {
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

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        if (frontLeft != null) frontLeft.setPower(fl / max);
        if (frontRight != null) frontRight.setPower(fr / max);
        if (backLeft != null) backLeft.setPower(bl / max);
        if (backRight != null) backRight.setPower(br / max);
    }

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }

    // --- VISION TARGET PROCESSING ---

    private static class TagChoice {
        final LLResultTypes.FiducialResult result;
        final int cellIndex;
        final double area;

        TagChoice(LLResultTypes.FiducialResult result, int cellIndex, double area) {
            this.result = result; this.cellIndex = cellIndex; this.area = area;
        }
    }

    private TagChoice processAprilTags(LLResult result, long now, double robotX, double robotY, double robotHeadingDeg) {
        TagChoice bestTag = null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;

        for (LLResultTypes.FiducialResult tag : fiducials) {
            int cellIndex = cellIndexForTag(tag.getFiducialId());
            if (cellIndex < 0) continue;

            cells[cellIndex].lastPositionUpdateMs = now;

            try {
                Pose3D targetPose = tag.getTargetPoseRobotSpace();
                if (targetPose != null && targetPose.getPosition() != null) {
                    Position relative = targetPose.getPosition().toUnit(DistanceUnit.INCH);
                    double headingRad = Math.toRadians(robotHeadingDeg);
                    double targetFieldX = robotX + relative.x * Math.cos(headingRad) + relative.y * Math.sin(headingRad);
                    double targetFieldY = robotY + relative.x * Math.sin(headingRad) - relative.y * Math.cos(headingRad);

                    CellTrack cell = cells[cellIndex];
                    if (!cell.hasFieldEstimate) {
                        cell.fieldXIn = targetFieldX; cell.fieldYIn = targetFieldY; cell.hasFieldEstimate = true;
                    } else {
                        cell.fieldXIn = 0.25 * targetFieldX + 0.75 * cell.fieldXIn;
                        cell.fieldYIn = 0.25 * targetFieldY + 0.75 * cell.fieldYIn;
                    }
                }
            } catch (Exception ignored) {}

            if (bestTag == null || tag.getTargetArea() > bestTag.area) {
                bestTag = new TagChoice(tag, cellIndex, tag.getTargetArea());
            }
        }
        return bestTag;
    }

    private int cellIndexForTag(int id) {
        for (int i = 0; i < Config.CELL_TAG_ID_MIN.length; i++) {
            if (id >= Config.CELL_TAG_ID_MIN[i] && id <= Config.CELL_TAG_ID_MAX[i]) return i;
        }
        return -1;
    }

    private void runAprilTagAlign(TagChoice visibleTag, long now, double robotX, double robotY, double robotHeadingDeg) {
        if (visibleTag == null) {
            stopDrive();
            assistStatus = "TAG LOST - SEARCHING";
            if (now - lastAlignTagSeenMs > 700) cancelAssist("TAG LOST");
            return;
        }

        lastAlignTagSeenMs = now;
        if (!alignGoalCaptured) {
            CellTrack targetCell = cells[visibleTag.cellIndex];
            if (!targetCell.hasFieldEstimate) {
                stopDrive(); assistStatus = "WAITING FOR 3D LOCATION"; return;
            }

            double dx = targetCell.fieldXIn - robotX, dy = targetCell.fieldYIn - robotY;
            double bearingRad = Math.hypot(dx, dy) > 4.0 ? Math.atan2(dy, dx) : Math.toRadians(robotHeadingDeg);

            alignGoalXIn = targetCell.fieldXIn - Config.TARGET_STANDOFF_IN * Math.cos(bearingRad);
            alignGoalYIn = targetCell.fieldYIn - Config.TARGET_STANDOFF_IN * Math.sin(bearingRad);
            alignGoalHeadingDeg = normalizeDegrees(Math.toDegrees(bearingRad));
            alignGoalCaptured = true;
        }

        double errorX = alignGoalXIn - robotX, errorY = alignGoalYIn - robotY;
        double distanceError = Math.hypot(errorX, errorY);
        double headingError = normalizeDegrees(alignGoalHeadingDeg - robotHeadingDeg);

        if (distanceError <= Config.POSITION_TOLERANCE_IN && Math.abs(headingError) <= Config.HEADING_TOLERANCE_DEG) {
            cancelAssist("ALIGNMENT COMPLETE");
            return;
        }

        double fieldXCmd = clip(errorX * Config.POSITION_KP, -Config.MAX_AUTO_TRANSLATION, Config.MAX_AUTO_TRANSLATION);
        double fieldYCmd = clip(errorY * Config.POSITION_KP, -Config.MAX_AUTO_TRANSLATION, Config.MAX_AUTO_TRANSLATION);
        double headingRad = Math.toRadians(robotHeadingDeg);

        double forward = fieldXCmd * Math.cos(headingRad) + fieldYCmd * Math.sin(headingRad);
        double strafeRight = fieldXCmd * Math.sin(headingRad) - fieldYCmd * Math.cos(headingRad);
        double turn = clip(-headingError * Config.HEADING_KP, -Config.MAX_AUTO_TURN, Config.MAX_AUTO_TURN);

        setRobotDrive(forward, strafeRight, turn);
        assistStatus = String.format(Locale.US, "ALIGNING: %.1fin, %.1f° err", distanceError, headingError);
    }

    private void runBlobApproach(BlobTarget target, long now) {
        if (target == null) {
            stopDrive();
            assistStatus = "BLOB LOST";
            if (now - lastBlobSeenMs > 700) cancelAssist("BLOB LOST");
            return;
        }

        lastBlobSeenMs = now;
        double strafeRight = clip(target.txDegrees * Config.BLOB_STRAFE_KP, -Config.MAX_AUTO_TRANSLATION, Config.MAX_AUTO_TRANSLATION);
        double forward = clip((Config.BLOB_STOP_AREA_PERCENT - target.areaPercent) * Config.BLOB_FORWARD_KP, 0.0, Config.MAX_AUTO_TRANSLATION);

        if (target.areaPercent >= Config.BLOB_STOP_AREA_PERCENT && Math.abs(target.txDegrees) <= Config.BLOB_CENTER_TOLERANCE_DEG) {
            cancelAssist("BLOB APPROACHED");
            return;
        }

        setRobotDrive(forward, strafeRight, 0.0);
        assistStatus = String.format(Locale.US, "BLOB tx %.1f°, area %.1f%%", target.txDegrees, target.areaPercent);
    }

    // --- TELEMETRY ---

    private void updateInitTelemetry() {
        double volts = batteryVoltageSensor != null ? batteryVoltageSensor.getVoltage() : 0.0;
        telemetry.addLine("=== BIOBUZZ TELEOP (ROBUST & EFFICIENT) ===");
        telemetry.addData("Battery Voltage", "%.2f V %s", volts, volts < 12.0 ? "(LOW BATTERY!)" : "(GOOD)");
        telemetry.addData("Alliance Target", Config.RED_ALLIANCE ? "RED Cells (IDs 30-37)" : "BLUE Cells (IDs 38-45)");
        telemetry.addData("Auto Align", Config.ENABLE_APRILTAG_AUTO_ALIGN ? "ENABLED" : "DISABLED");
        telemetry.addData("Blob Approach", Config.ENABLE_BLOB_AUTO_APPROACH ? "ENABLED" : "DISABLED");

        telemetry.addLine("\n--- SENSOR READINESS ---");
        telemetry.addData("Limelight", limelight != null && limelight.isConnected() ? "CONNECTED" : "DISCONNECTED");
        telemetry.addData("Webcam", webcamStatus);
        telemetry.addData("Pinpoint Pose", pinpoint != null ? String.format(Locale.US, "X: %.1f, Y: %.1f, H: %.1f°",
                pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(AngleUnit.DEGREES)) : "DISCONNECTED");

        telemetry.addLine("\n--- CONTROLS ---");
        telemetry.addLine("  [X Button] Toggle Alliance Color");
        telemetry.addLine("  [A] AprilTag Auto Align | [B] Blob Approach");
        telemetry.addLine("  [Y / Options] Re-Zero Field Heading");
        telemetry.addLine("  [RB + D-Pad] Wheel Direction Test Mode");
        telemetry.update();
    }

    private void updateTelemetry(double robotX, double robotY, double robotHeadingDeg, TagChoice bestTag, BlobTarget blobTarget) {
        double volts = batteryVoltageSensor != null ? batteryVoltageSensor.getVoltage() : 0.0;
        telemetry.addLine("=== DRIVE & STATUS ===");
        telemetry.addData("Mode", assistStatus);
        telemetry.addData("Alliance / Battery", "%s | %.2f V", Config.RED_ALLIANCE ? "RED" : "BLUE", volts);

        telemetry.addLine("\n=== ROBOT POSE ===");
        telemetry.addData("Position", "X: %5.1f in | Y: %5.1f in | H: %5.1f°", robotX, robotY, robotHeadingDeg);

        telemetry.addLine("\n=== VISION ===");
        telemetry.addData("Tag Target", bestTag != null ? String.format(Locale.US, "%s (#%d) tx: %+.1f°", cells[bestTag.cellIndex].name, bestTag.result.getFiducialId(), bestTag.result.getTargetXDegrees()) : "None");
        telemetry.addData("Blob Target", blobTarget != null ? String.format(Locale.US, "%s tx: %+.1f°, area: %.1f%%", blobTarget.label, blobTarget.txDegrees, blobTarget.areaPercent) : "None");

        telemetry.update();
    }

    // --- UTILITIES & PIPELINE ---

    private static double clip(double val, double low, double high) { return Math.max(low, Math.min(high, val)); }
    private static double normalizeDegrees(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private class WebcamBlobPipeline extends OpenCvPipeline {
        private final Mat rgb = new Mat(), hsv = new Mat(), mask = new Mat(), hierarchy = new Mat();
        private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));
        private final List<MatOfPoint> contours = new java.util.ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {
            try {
                Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);
                Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
                Core.inRange(hsv, new Scalar(Config.HSV_H_MIN, Config.HSV_S_MIN, Config.HSV_V_MIN),
                        new Scalar(Config.HSV_H_MAX, Config.HSV_S_MAX, Config.HSV_V_MAX), mask);
                Imgproc.erode(mask, mask, kernel, new org.opencv.core.Point(-1, -1), 1);
                Imgproc.dilate(mask, mask, kernel, new org.opencv.core.Point(-1, -1), 2);

                contours.clear();
                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                double bestArea = 0.0; Rect bestRect = null;
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > bestArea) { bestArea = area; bestRect = Imgproc.boundingRect(contour); }
                    contour.release();
                }
                contours.clear();

                if (bestRect != null && bestArea >= Config.MIN_BLOB_AREA_PX) {
                    double centerX = bestRect.x + bestRect.width / 2.0;
                    double imageCenterX = input.cols() / 2.0;
                    double tx = ((centerX - imageCenterX) / imageCenterX) * 30.0;
                    double areaPercent = 100.0 * bestArea / (input.cols() * (double) input.rows());
                    latestWebcamBlob = new BlobTarget("Color Blob", tx, areaPercent);

                    Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
                } else {
                    latestWebcamBlob = null;
                }
            } catch (Exception ignored) {}
            latestWebcamBlobMs = System.currentTimeMillis();
            return input;
        }
    }

    private static class BlobTarget {
        final String label;
        final double txDegrees, areaPercent;

        BlobTarget(String label, double txDegrees, double areaPercent) {
            this.label = label; this.txDegrees = txDegrees; this.areaPercent = areaPercent;
        }
    }
}
