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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * BIOBUZZ TeleOp
 *
 * Controls (Gamepad 1):
 *  - Sticks: field-centric mecanum drive
 *  - Y / Options: re-zero field heading
 *  - A: AprilTag align to one of YOUR alliance's HIVE CELLS (toggle, sticks cancel it)
 *  - B: drive toward the yellow POLLEN (toggle, sticks cancel it)
 *  - X (during init): switch RED / BLUE alliance
 *  - RB + D-Pad: single-wheel direction test
 *
 * BIOBUZZ AprilTag layout (36h11, 3.25 in tags, 4 tags per CELL):
 *  - Red  far  CELL (opposite audience): IDs 30-33
 *  - Red  audience CELL:                 IDs 34-37
 *  - Blue audience CELL:                 IDs 38-41
 *  - Blue far  CELL (opposite audience): IDs 42-45
 * The tag sticker faces down at the floor, so mount the Limelight where it can see UP/forward at the cells.
 *
 * Limelight setup: AprilTag pipeline, family 36h11, tag size 82.5 mm (3.25 in).
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BASE extends LinearOpMode {

    @com.acmerobotics.dashboard.config.Config
    public static class Config {
        // ---------- Hardware names (must match the Robot Configuration) ----------
        public static String FRONT_LEFT_NAME = "front_left";
        public static String FRONT_RIGHT_NAME = "front_right";
        public static String BACK_LEFT_NAME = "back_left";
        public static String BACK_RIGHT_NAME = "back_right";
        public static String PINPOINT_NAME = "pinpoint";
        public static String LIMELIGHT_NAME = "limelight";
        public static String WEBCAM_NAME = "Webcam 1";

        // ---------- Motor reversals (use RB + D-Pad test to check) ----------
        public static boolean REVERSE_FRONT_LEFT = false;
        public static boolean REVERSE_FRONT_RIGHT = true;
        public static boolean REVERSE_BACK_LEFT = false;
        public static boolean REVERSE_BACK_RIGHT = true;

        // ---------- Drive direction ----------
        public static boolean INVERT_FORWARD = true; // flips forward/back for sticks AND auto assists

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
        public static double MAX_AUTO_TRANSLATION = 0.20;
        public static double MAX_AUTO_TURN = 0.12;

        // ---------- AprilTag align ----------
        public static int APRILTAG_PIPELINE = 0;           // Limelight pipeline number for AprilTags
        public static double TARGET_STANDOFF_IN = 36.0;    // distance from camera/robot to the tags to stop at (TUNE THIS)
        public static String ALIGN_TAG_IDS = "31,35,39,43";   // tags auto-align may use, comma separated (add your 4th here)
        public static double ALIGN_TX_OFFSET_DEG = 0.0;    // left/right offset: + = robot ends aimed LEFT of the tag, - = RIGHT
        public static double ALIGN_DISTANCE_OFFSET_IN = 0.0; // added to TARGET_STANDOFF_IN: + = stop farther back
        public static double MIN_AUTO_TURN = 0.08;          // smallest turn power it uses (raise if it stalls short)
        public static double MIN_AUTO_FORWARD = 0.10;       // smallest drive power it uses (raise if it stalls short)
        public static long ALIGN_TIMEOUT_MS = 4000;         // auto align stops by itself after this long
        public static boolean REVERSE_ALIGN_TURN = true;    // flip if auto align turns AWAY from the tag
        public static boolean REVERSE_ALIGN_FORWARD = true; // flip if auto align drives AWAY from the tag
        public static double POSITION_KP = 0.02;           // forward power per inch of error
        public static double HEADING_KP = 0.015;           // turn power per degree of tx
        public static double POSITION_TOLERANCE_IN = 2.0;
        public static double HEADING_TOLERANCE_DEG = 2.0;

        // ---------- Yellow POLLEN blob (webcam) ----------
        public static int HSV_H_MIN = 20;
        public static int HSV_H_MAX = 38;
        public static int HSV_S_MIN = 80;
        public static int HSV_S_MAX = 255;
        public static int HSV_V_MIN = 60;
        public static int HSV_V_MAX = 255;
        public static double MIN_BLOB_AREA_PX = 350.0;
        public static double BLOB_STOP_AREA_PERCENT = 4.0;
        public static double BLOB_FORWARD_KP = 0.055;
        public static double BLOB_STRAFE_KP = 0.014;
        public static double BLOB_CENTER_TOLERANCE_DEG = 2.5;
    }

    // ---------- BIOBUZZ tag layout (from the game manual) ----------
    private static final int FIRST_TAG_ID = 30;
    private static final int LAST_TAG_ID = 45;
    // Cell index = (id - 30) / 4  ->  0 red far, 1 red audience, 2 blue audience, 3 blue far
    private static final String[] CELL_NAMES = {
            "Red Far Cell", "Red Audience Cell", "Blue Audience Cell", "Blue Far Cell"
    };

    // ---------- Hardware ----------
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private OpenCvWebcam webcam;
    private VoltageSensor batteryVoltageSensor;

    // ---------- Webcam blob ----------
    private final WebcamBlobPipeline webcamBlobPipeline = new WebcamBlobPipeline();
    private volatile BlobTarget latestWebcamBlob;
    private volatile long latestWebcamBlobMs = 0;
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
    private String tagDebug = "";
    private boolean alignTagSeen = false;
    private long alignStartMs = 0;
    private String alignDebug = "";
    private long lastAlignTagSeenMs = 0, lastBlobSeenMs = 0;

    /** Robot heading in degrees (counter-clockwise positive), flipped if REVERSE_HEADING is on. */
    private double headingDeg() {
        if (pinpoint == null) return 0.0;
        double h = pinpoint.getHeading(AngleUnit.DEGREES);
        return Config.REVERSE_HEADING ? -h : h;
    }

    @Override
    public void runOpMode() {
        initializeHardware();
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
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !previousInitX) {
                Config.RED_ALLIANCE = !Config.RED_ALLIANCE;
            }
            previousInitX = gamepad1.x;

            updatePinpointConfigIfNeeded();
            updateWebcamExposureIfNeeded();
            if (pinpoint != null) {
                pinpoint.update();
                // The way the robot is facing when you press START becomes "forward" for the driver
                fieldHeadingOffsetDeg = headingDeg();
            }

            updateInitTelemetry();
            sleep(20);
        }

        // ---------- MAIN LOOP ----------
        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            updatePinpointConfigIfNeeded();
            updateWebcamExposureIfNeeded();

            double robotX = 0.0, robotY = 0.0, robotHeadingDeg = 0.0;
            if (pinpoint != null) {
                pinpoint.update();
                robotX = pinpoint.getPosX(DistanceUnit.INCH);
                robotY = pinpoint.getPosY(DistanceUnit.INCH);
                robotHeadingDeg = headingDeg();
            }

            handleButtons(now);

            boolean manualOverride = hasManualStickInput();
            if (manualOverride && (autoAlignActive || blobApproachActive)) {
                cancelAssist("MANUAL OVERRIDE");
            }

            TagView tagView = readAprilTags();
            BlobTarget blobTarget = (latestWebcamBlob != null && now - latestWebcamBlobMs <= 700)
                    ? latestWebcamBlob : null;

            if (autoAlignActive) {
                runAprilTagAlign(tagView, now);
            } else if (blobApproachActive) {
                runBlobApproach(blobTarget, now);
            } else {
                boolean wheelTest = runFieldCentricDrive(robotHeadingDeg);
                if (!wheelTest && !assistStatus.equals("HEADING RE-ZEROED")) {
                    assistStatus = manualOverride ? "MANUAL DRIVE" : "READY";
                }
            }

            updateTelemetry(robotX, robotY, robotHeadingDeg, tagView, blobTarget);
            idle();
        }

        // ---------- SHUTDOWN ----------
        stopDrive();
        if (limelight != null) limelight.stop();
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    // =====================================================================
    // SETUP
    // =====================================================================

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
            telemetry.update();
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
                alignStartMs = now;
                lastAlignTagSeenMs = now;
                assistStatus = "LOOKING FOR CELL TAGS";
            }
        }

        if (pressedB) {
            if (blobApproachActive) {
                cancelAssist("BLOB CANCELLED");
            } else if (!Config.ENABLE_BLOB_AUTO_APPROACH) {
                assistStatus = "BLOB DISABLED (Config)";
            } else {
                autoAlignActive = false;
                blobApproachActive = true;
                lastBlobSeenMs = now;
                assistStatus = "LOOKING FOR POLLEN";
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

        boolean pressedLeftBumper = gamepad1.left_bumper && !previousLeftBumper;
        if (pressedLeftBumper) {
            Config.FIELD_CENTRIC = !Config.FIELD_CENTRIC;
            assistStatus = Config.FIELD_CENTRIC ? "FIELD-CENTRIC ON" : "ROBOT-CENTRIC ON";
        }
        previousLeftBumper = gamepad1.left_bumper;

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
        assistStatus = reason;
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
        if (Config.INVERT_FORWARD) {
            forward = -forward;
            strafeRight = -strafeRight;
        }
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

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);
    }

    // =====================================================================
    // APRILTAG ALIGN (BIOBUZZ CELLS)
    // =====================================================================

    /** What the Limelight sees on one CELL (tags averaged together). */
    private static class TagView {
        final int tagId;
        final double txDeg;       // + means the cell is to the right
        final double distanceIn;  // NaN if unknown
        final int tagCount;

        TagView(int tagId, double txDeg, double distanceIn, int tagCount) {
            this.tagId = tagId;
            this.txDeg = txDeg;
            this.distanceIn = distanceIn;
            this.tagCount = tagCount;
        }
    }

    private static int cellIndexForTag(int id) {
        if (id < FIRST_TAG_ID || id > LAST_TAG_ID) return -1;
        return (id - FIRST_TAG_ID) / 4;
    }

    /** Red owns cells 0 and 1 (IDs 30-37). Blue owns cells 2 and 3 (IDs 38-45). */
    private static boolean isOwnAlliance(int cellIndex) {
        return (cellIndex / 2) == (Config.RED_ALLIANCE ? 0 : 1);
    }

    private int[] allowedTagIds = new int[0];
    private String allowedTagIdsSource = null;

    /** True if this tag ID is in Config.ALIGN_TAG_IDS (comma separated list). */
    private boolean isAllowedTag(int id) {
        if (!Config.ALIGN_TAG_IDS.equals(allowedTagIdsSource)) {
            allowedTagIdsSource = Config.ALIGN_TAG_IDS;
            String[] parts = allowedTagIdsSource.split(",");
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
        if (limelight.getTimeSinceLastUpdate() > 250) {
            tagDebug = "STALE DATA";
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            tagDebug = "NO TAGS IN RESULT";
            return null;
        }

        StringBuilder seen = new StringBuilder();
        LLResultTypes.FiducialResult best = null;
        int allowedCount = 0;

        for (LLResultTypes.FiducialResult tag : fiducials) {
            seen.append(tag.getFiducialId()).append(' ');
            if (!isAllowedTag(tag.getFiducialId())) continue;
            allowedCount++;
            // if more than one allowed tag is visible, use the biggest (closest) one
            if (best == null || tag.getTargetArea() > best.getTargetArea()) best = tag;
        }

        if (best == null) {
            tagDebug = "SEES TAG IDs [" + seen.toString().trim() + "] BUT NONE ARE IN ALIGN_TAG_IDS ("
                    + Config.ALIGN_TAG_IDS + ")";
            return null;
        }
        tagDebug = "OK - IDs [" + seen.toString().trim() + "]";

        return new TagView(best.getFiducialId(), best.getTargetXDegrees(), tagDistanceIn(best), allowedCount);
    }

    /** Straight-line distance to the tag, in inches. Works for any Limelight axis layout. */
    private static double tagDistanceIn(LLResultTypes.FiducialResult tag) {
        try {
            Pose3D pose = tag.getTargetPoseRobotSpace();
            if (pose == null || pose.getPosition() == null) return Double.NaN;
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            double d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            return d > 0.5 ? d : Double.NaN;
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    private void runAprilTagAlign(TagView tag, long now) {
        if (now - alignStartMs > Config.ALIGN_TIMEOUT_MS) {
            cancelAssist("ALIGN TIMEOUT");
            return;
        }
        if (tag == null) {
            stopDrive();
            long timeoutMs = alignTagSeen ? 700 : 5000; // give you 5 sec to aim at the tags first
            assistStatus = alignTagSeen ? "TAG LOST - SEARCHING" : "NO TAG YET - POINT CAMERA AT CELL";
            if (now - lastAlignTagSeenMs > timeoutMs) {
                cancelAssist(alignTagSeen ? "TAG LOST" : "NO TAG FOUND (see Tag Debug)");
            }
            return;
        }
        alignTagSeen = true;
        lastAlignTagSeenMs = now;

        double txError = tag.txDeg - Config.ALIGN_TX_OFFSET_DEG;
        boolean aimed = Math.abs(txError) <= Config.HEADING_TOLERANCE_DEG;
        double turn = 0.0;
        if (!aimed) {
            turn = clip(txError * Config.HEADING_KP, -Config.MAX_AUTO_TURN, Config.MAX_AUTO_TURN);
            // small errors give power too weak to move the robot, so use a minimum push
            turn = Math.copySign(Math.max(Math.abs(turn), Config.MIN_AUTO_TURN), turn);
        }

        double forward = 0.0;
        boolean rangeOk = true;
        double distErr = 0.0;
        if (!Double.isNaN(tag.distanceIn)) {
            distErr = tag.distanceIn - (Config.TARGET_STANDOFF_IN + Config.ALIGN_DISTANCE_OFFSET_IN);
            rangeOk = Math.abs(distErr) <= Config.POSITION_TOLERANCE_IN;
            if (!rangeOk) {
                forward = clip(distErr * Config.POSITION_KP,
                        -Config.MAX_AUTO_TRANSLATION, Config.MAX_AUTO_TRANSLATION);
                forward = Math.copySign(Math.max(Math.abs(forward), Config.MIN_AUTO_FORWARD), forward);
            }
        }

        if (aimed && rangeOk) {
            cancelAssist("ALIGNED TO TAG " + tag.tagId);
            return;
        }

        alignDebug = String.format(Locale.US, "tx %.1f (offset %.1f) -> turn cmd %.2f | dist %.1f vs target %.1f -> fwd cmd %.2f",
                tag.txDeg, Config.ALIGN_TX_OFFSET_DEG, turn, tag.distanceIn,
                Config.TARGET_STANDOFF_IN + Config.ALIGN_DISTANCE_OFFSET_IN, forward);
        if (Config.REVERSE_ALIGN_TURN) turn = -turn;
        if (Config.REVERSE_ALIGN_FORWARD) forward = -forward;
        setRobotDrive(forward, 0.0, turn);
        assistStatus = String.format(Locale.US, "ALIGNING TAG %d: aim err %.1f°, range err %.1f in",
                tag.tagId, txError, distErr);
    }

    // =====================================================================
    // POLLEN BLOB APPROACH
    // =====================================================================

    private void runBlobApproach(BlobTarget target, long now) {
        if (target == null) {
            stopDrive();
            assistStatus = "POLLEN LOST";
            if (now - lastBlobSeenMs > 700) cancelAssist("POLLEN LOST");
            return;
        }
        lastBlobSeenMs = now;

        if (target.areaPercent >= Config.BLOB_STOP_AREA_PERCENT
                && Math.abs(target.txDegrees) <= Config.BLOB_CENTER_TOLERANCE_DEG) {
            cancelAssist("POLLEN REACHED");
            return;
        }

        double strafeRight = clip(target.txDegrees * Config.BLOB_STRAFE_KP,
                -Config.MAX_AUTO_TRANSLATION, Config.MAX_AUTO_TRANSLATION);
        double forward = clip((Config.BLOB_STOP_AREA_PERCENT - target.areaPercent) * Config.BLOB_FORWARD_KP,
                0.0, Config.MAX_AUTO_TRANSLATION);

        setRobotDrive(forward, strafeRight, 0.0);
        assistStatus = String.format(Locale.US, "POLLEN tx %.1f°, area %.1f%%",
                target.txDegrees, target.areaPercent);
    }

    // =====================================================================
    // TELEMETRY
    // =====================================================================

    private void updateInitTelemetry() {
        double volts = batteryVoltageSensor != null ? batteryVoltageSensor.getVoltage() : 0.0;
        telemetry.addLine("=== BIOBUZZ TELEOP ===");
        telemetry.addData("Battery", "%.2f V %s", volts, volts < 12.0 ? "(LOW!)" : "(GOOD)");
        telemetry.addData("Alliance", Config.RED_ALLIANCE ? "RED (cell tags 30-37)" : "BLUE (cell tags 38-45)");
        telemetry.addData("Tag Align (A)", Config.ENABLE_APRILTAG_AUTO_ALIGN ? "ENABLED" : "DISABLED");
        telemetry.addData("Pollen Approach (B)", Config.ENABLE_BLOB_AUTO_APPROACH ? "ENABLED" : "DISABLED");

        telemetry.addLine("--- SENSORS ---");
        telemetry.addData("Limelight", limelight != null && limelight.isConnected() ? "CONNECTED" : "DISCONNECTED");
        telemetry.addData("Webcam", webcamStatus);
        telemetry.addData("Pinpoint", pinpoint != null
                ? String.format(Locale.US, "X %.1f, Y %.1f, H %.1f°",
                pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                headingDeg())
                : "DISCONNECTED");

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("[X] Toggle alliance | [A] Align to cell | [B] Pollen");
        telemetry.addLine("[Y / Options] Re-zero heading | [LB] Field/Robot-centric | [RB + D-Pad] Wheel test");
        telemetry.update();
    }

    private void updateTelemetry(double robotX, double robotY, double robotHeadingDeg,
                                 TagView tag, BlobTarget blob) {
        double volts = batteryVoltageSensor != null ? batteryVoltageSensor.getVoltage() : 0.0;
        telemetry.addData("Mode", assistStatus);
        telemetry.addData("Drive", !Config.FIELD_CENTRIC ? "ROBOT-CENTRIC"
                : (pinpoint == null ? "FIELD-CENTRIC (NO PINPOINT - acting robot-centric)" : "FIELD-CENTRIC"));
        telemetry.addData("Heading vs Zero", "%.1f°", normalizeDegrees(robotHeadingDeg - fieldHeadingOffsetDeg));
        telemetry.addData("Alliance / Battery", "%s | %.2f V", Config.RED_ALLIANCE ? "RED" : "BLUE", volts);
        telemetry.addData("Pose", "X %5.1f | Y %5.1f | H %5.1f°", robotX, robotY, robotHeadingDeg);
        telemetry.addData("Cell Tags", tag != null
                ? String.format(Locale.US, "Tag #%d (%d allowed in view) tx %+.1f°, dist %.1f in",
                tag.tagId, tag.tagCount, tag.txDeg, tag.distanceIn)
                : "None");
        telemetry.addData("Tag Debug", tagDebug);
        telemetry.addData("Align Debug", alignDebug);
        telemetry.addData("Pollen", blob != null
                ? String.format(Locale.US, "tx %+.1f°, area %.1f%%", blob.txDegrees, blob.areaPercent)
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

    private static class BlobTarget {
        final String label;
        final double txDegrees, areaPercent;

        BlobTarget(String label, double txDegrees, double areaPercent) {
            this.label = label;
            this.txDegrees = txDegrees;
            this.areaPercent = areaPercent;
        }
    }

    private class WebcamBlobPipeline extends OpenCvPipeline {
        private final Mat rgb = new Mat(), hsv = new Mat(), mask = new Mat(), hierarchy = new Mat();
        private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));
        private final List<MatOfPoint> contours = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {
            try {
                Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);
                Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
                Core.inRange(hsv,
                        new Scalar(Config.HSV_H_MIN, Config.HSV_S_MIN, Config.HSV_V_MIN),
                        new Scalar(Config.HSV_H_MAX, Config.HSV_S_MAX, Config.HSV_V_MAX), mask);
                Imgproc.erode(mask, mask, kernel, new org.opencv.core.Point(-1, -1), 1);
                Imgproc.dilate(mask, mask, kernel, new org.opencv.core.Point(-1, -1), 2);

                contours.clear();
                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
                    double tx = ((centerX - imageCenterX) / imageCenterX) * 30.0;
                    double areaPercent = 100.0 * bestArea / (input.cols() * (double) input.rows());
                    latestWebcamBlob = new BlobTarget("Pollen", tx, areaPercent);
                    latestWebcamBlobMs = System.currentTimeMillis();

                    Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
                } else {
                    latestWebcamBlob = null;
                }
            } catch (Exception ignored) {
                latestWebcamBlob = null;
            }
            return input;
        }
    }
}