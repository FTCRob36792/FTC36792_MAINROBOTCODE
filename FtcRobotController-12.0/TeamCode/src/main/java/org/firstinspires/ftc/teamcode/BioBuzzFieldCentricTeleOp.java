package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;
import java.util.Locale;

/**
 * BIOBUZZ TeleOp starter:
 * - Field-centric mecanum drive, using the Pinpoint heading.
 * - Continuously reads Pinpoint X/Y/heading.
 * - Tracks the BIOBUZZ AprilTag cell groups and last estimated field locations.
 * - Limelight 3 runs its AprilTag pipeline continuously for shooting-target tracking.
 * - Logitech webcam runs an independent EasyOpenCV HSV color-blob pipeline concurrently.
 * - A: start/stop AprilTag-based alignment to the currently observed open cell for this alliance.
 * - B: start/stop approaching the largest visible webcam blob.
 * - Manual stick input cancels either assist mode immediately.
 *
 * IMPORTANT: Set all hardware names, starting pose, Pinpoint offsets/directions,
 * Limelight camera transform, HSV bounds, and webcam field of view before robot testing.
 */
@TeleOp(name = "BIOBUZZ Field-Centric Vision TeleOp", group = "BIOBUZZ")
public class BioBuzzFieldCentricTeleOp extends LinearOpMode {

    // =====================================================================
    // ALL ROBOT-SPECIFIC CONFIGURATION / TUNING PARAMETERS LIVE HERE.
    // Change values in this section first; avoid tuning magic numbers elsewhere.
    // Any value marked VERIFY or CALIBRATE must be checked on your actual robot.
    // Automatic assists are DISABLED by default until sensor and motion tests pass.
    // =====================================================================
    private static final class Config {
        private Config() { }

        // [VERIFY] Exact names from the Driver Station Robot Configuration.
        static final String FRONT_LEFT_NAME = "front_left";
        static final String FRONT_RIGHT_NAME = "front_right";
        static final String BACK_LEFT_NAME = "back_left";
        static final String BACK_RIGHT_NAME = "back_right";
        static final String PINPOINT_NAME = "pinpoint";
        static final String LIMELIGHT_NAME = "limelight";
        static final String WEBCAM_NAME = "Webcam 1";

        // [VERIFY] Limelight pipeline index configured for AprilTags.
        static final int APRILTAG_PIPELINE = 0;
        static final int LIMELIGHT_POLL_RATE_HZ = 100;
        static final long MAX_RESULT_AGE_MS = 250;

        // [VERIFY] Game-specific tag-ID groups. These are assumptions from the starter code;
        // confirm against the official game field/tag layout before relying on target selection.
        static final int[] CELL_TAG_ID_MIN = {30, 34, 38, 42};
        static final int[] CELL_TAG_ID_MAX = {33, 37, 41, 45};

        // [SET] True for RED, false for BLUE.
        static final boolean RED_ALLIANCE = true;

        // [MEASURE] Actual robot starting pose in the Pinpoint field coordinate system.
        static final double START_X_IN = 0.0;
        static final double START_Y_IN = 0.0;
        static final double START_HEADING_DEG = 0.0;

        // [MEASURE] Pinpoint pod offsets in millimeters relative to robot tracking center.
        // Use the Pinpoint documentation's sign convention; zero is only correct if measured.
        static final double PINPOINT_X_POD_OFFSET_MM = 0.0;
        static final double PINPOINT_Y_POD_OFFSET_MM = 0.0;
        // [VERIFY] Change pod type if your robot uses swingarm pods.
        static final GoBildaPinpointDriver.GoBildaOdometryPods ODOMETRY_POD_TYPE =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        // [CALIBRATE] Encoder directions must match measured positive X/Y motion.
        static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;

        // [VERIFY] Motor direction configuration. Test each wheel with the robot safely lifted.
        static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

        // [VERIFY] Camera image format/orientation. Resolution is used for the webcam stream.
        static final int WEBCAM_WIDTH = 640;
        static final int WEBCAM_HEIGHT = 480;
        static final OpenCvCameraRotation WEBCAM_ROTATION = OpenCvCameraRotation.UPRIGHT;
        // [CALIBRATE] Actual horizontal FOV at the selected resolution and camera mode.
        static final double WEBCAM_HORIZONTAL_FOV_DEG = 60.0;

        // [CALIBRATE] HSV color threshold for the target element under your lighting.
        // OpenCV hue range is 0..179; saturation and value ranges are 0..255.
        static final int HSV_H_MIN = 20;
        static final int HSV_H_MAX = 38;
        static final int HSV_S_MIN = 80;
        static final int HSV_S_MAX = 255;
        static final int HSV_V_MIN = 60;
        static final int HSV_V_MAX = 255;
        // [CALIBRATE] Minimum contour area to reject small noise (pixels squared).
        static final double MIN_BLOB_AREA_PX = 350.0;

        // [SAFETY] Leave false until each assist has been tested and calibrated independently.
        static final boolean ENABLE_APRILTAG_AUTO_ALIGN = false;
        static final boolean ENABLE_BLOB_AUTO_APPROACH = false;

        // [TUNE] Driver response and automatic-drive power caps (0.0 to 1.0).
        static final double STICK_DEADZONE = 0.16;
        static final double MAX_AUTO_TRANSLATION = 0.20;
        static final double MAX_AUTO_TURN = 0.12;

        // [MEASURE/TUNE] Shooting alignment geometry and proportional controller.
        static final double TARGET_STANDOFF_IN = 36.0;
        static final double POSITION_KP = 0.025; // motor power per inch of field-position error
        static final double HEADING_KP = 0.012;  // motor power per degree of heading error
        static final double POSITION_TOLERANCE_IN = 2.5;
        static final double HEADING_TOLERANCE_DEG = 3.0;
        // [CALIBRATE] Camera/tag roll baseline and classification threshold.
        // The roll-based open/closed heuristic is experimental, not validated game logic.
        static final double CAMERA_TAG_ROLL_OFFSET_DEG = 0.0;
        static final double CELL_UPRIGHT_ROLL_LIMIT_DEG = 90.0;
        // [TUNE] Prevent a noisy single-frame position estimate from defining the target.
        static final double MIN_DISTANCE_FOR_BEARING_IN = 4.0;
        static final double CELL_POSITION_SMOOTHING_ALPHA = 0.25;

        // [CALIBRATE] Blob approach. Area is image percentage, NOT a calibrated distance.
        static final double BLOB_STOP_AREA_PERCENT = 4.0;
        static final double BLOB_FORWARD_KP = 0.055;
        static final double BLOB_STRAFE_KP = 0.014;
        static final double BLOB_CENTER_TOLERANCE_DEG = 2.5;

        // [TUNE] Timeouts and telemetry cadence in milliseconds.
        static final long TARGET_STATE_TIMEOUT_MS = 1500;
        static final long VISION_LOSS_TIMEOUT_MS = 700;
        static final int TELEMETRY_INTERVAL_MS = 50;
    }

    // ------------------------------ Hardware ------------------------------
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private OpenCvCamera webcam;
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
        // Do not switch Limelight pipelines: it continuously observes AprilTags while
        // the Logitech webcam independently runs the OpenCV blob detector.
        limelight.pipelineSwitch(Config.APRILTAG_PIPELINE);
        initializeWebcam();

        telemetry.setMsTransmissionInterval(Config.TELEMETRY_INTERVAL_MS);
        telemetry.addLine("BIOBUZZ Field-Centric Vision TeleOp");
        telemetry.addLine("Check hardware names, Pinpoint offsets, starting pose, camera pose, and pipeline indices.");
        telemetry.addData("Alliance target", Config.RED_ALLIANCE ? "RED cells (IDs 30-37)" : "BLUE cells (IDs 38-45)");
        telemetry.addData("Starting pose", "X %.1f in, Y %.1f in, H %.1f deg",
                Config.START_X_IN, Config.START_Y_IN, Config.START_HEADING_DEG);
        telemetry.addLine("A = AprilTag align; B = blob approach; stick input cancels assist.");
        telemetry.addData("Auto align enabled", Config.ENABLE_APRILTAG_AUTO_ALIGN);
        telemetry.addData("Blob approach enabled", Config.ENABLE_BLOB_AUTO_APPROACH);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

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
                assistStatus = manualOverride ? "MANUAL DRIVE" : "READY";
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

        // Common mirrored mecanum setup; change directions if the wheels do not all push forward.
        frontLeft.setDirection(Config.FRONT_LEFT_DIRECTION);
        backLeft.setDirection(Config.BACK_LEFT_DIRECTION);
        frontRight.setDirection(Config.FRONT_RIGHT_DIRECTION);
        backRight.setDirection(Config.BACK_RIGHT_DIRECTION);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Config.PINPOINT_NAME);
        limelight = hardwareMap.get(Limelight3A.class, Config.LIMELIGHT_NAME);
    }

    private void initializeWebcam() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, Config.WEBCAM_NAME);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        webcam.setPipeline(webcamBlobPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Config.WEBCAM_WIDTH, Config.WEBCAM_HEIGHT, Config.WEBCAM_ROTATION);
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
        pinpoint.setEncoderDirections(Config.X_ENCODER_DIRECTION, Config.Y_ENCODER_DIRECTION);

        // This sets the Pinpoint's pose to the robot's actual field location at match start.
        // The Pinpoint guide uses field origin at center; +X toward audience; +Y toward alliance.
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                Config.START_X_IN,
                Config.START_Y_IN,
                AngleUnit.DEGREES,
                Config.START_HEADING_DEG
        ));
    }

    private void handleButtons(long now) {
        boolean pressedA = gamepad1.a && !previousA;
        boolean pressedB = gamepad1.b && !previousB;

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

        previousA = gamepad1.a;
        previousB = gamepad1.b;
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

                // The tag cluster on the upward-facing BIOBUZZ cell should appear upright when
                // the camera is aimed along the launcher direction. Limelight returns each tag
                // separately, so use camera-space pose roll as a first-pass orientation signal.
                // The sign convention must be validated on the real field/camera mount.
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
                    // Keep cell state UNKNOWN if no usable orientation is available.
                }

                TagChoice oldChoice = bestTagForCell[cellIndex];
                if (oldChoice == null || tag.getTargetArea() > oldChoice.area) {
                    bestTagForCell[cellIndex] = new TagChoice(tag, cellIndex, tag.getTargetArea());
                }

                // TargetPoseRobotSpace requires correct Limelight robot-space mounting settings
                // (LL Forward/Right/Up and Roll/Pitch/Yaw) and full 3D AprilTag tracking.
                try {
                    Pose3D targetPose = tag.getTargetPoseRobotSpace();
                    if (targetPose != null && targetPose.getPosition() != null) {
                        Position relative = targetPose.getPosition().toUnit(DistanceUnit.INCH);
                        double targetForward = relative.x;
                        double targetRight = relative.y; // Limelight robot-space convention: right is positive.
                        double headingRad = Math.toRadians(robotHeadingDeg);

                        // Pinpoint field axes: +X toward audience, +Y toward alliance.
                        // Robot forward = (cos(h), sin(h)); robot right = (sin(h), -cos(h)).
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
                    // If this result does not contain a usable 3D pose, keep the last known location.
                }
            }
        }

        // Update each cell's tag-location estimate. If multiple IDs in one cluster are visible,
        // averaging reduces the offset from an individual AprilTag toward the cluster center.
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

        // A visible tag alone does NOT prove that its cell is open: the opposing cell's tag
        // can also be visible. The BIOBUZZ scoring cell is identified by tag-cluster orientation.
        // Upright tag -> this cell is open; upside-down tag -> its partner is open.
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

    // Webcam blob target is produced asynchronously by WebcamBlobPipeline.
    // It is independent of Limelight's AprilTag observations.

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

            // Capture a fixed goal at a tunable horizontal stand-off distance from the target.
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

        // Proportional field-coordinate position control, converted to robot forward/right.
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
        // With the common mecanum wheel signs below, negative turn power corrects a positive
        // counter-clockwise heading error. Verify this on blocks and reverse this sign if needed.
        double turn = clip(-headingError * Config.HEADING_KP, -Config.MAX_AUTO_TURN, Config.MAX_AUTO_TURN);

        setRobotDrive(forward, strafeRight, turn);
        assistStatus = String.format(Locale.US,
                "ALIGN %s: %.1f in, heading error %.1f deg",
                cells[alignCellIndex].name, distanceError, headingError);
    }

    // ------------------------------- Driving ------------------------------

    private void runFieldCentricDrive(double robotHeadingDeg) {
        // Driver forward/right are relative to the heading specified for the starting pose,
        // so pushing the stick up continues to mean "away from our starting wall".
        double fieldRight = gamepad1.left_stick_x;
        double fieldForward = -gamepad1.left_stick_y;
        double relativeHeading = Math.toRadians(normalizeDegrees(robotHeadingDeg - Config.START_HEADING_DEG));

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

    private void updateTelemetry(long now,
                                 double robotX,
                                 double robotY,
                                 double robotHeadingDeg,
                                 TagChoice bestAllianceTag,
                                 BlobTarget blobTarget) {
        telemetry.addData("Mode", assistStatus);
        telemetry.addData("Alliance", Config.RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.addData("Limelight connected", limelight.isConnected());
        telemetry.addData("Limelight pipeline", "AprilTag %d (continuous)", Config.APRILTAG_PIPELINE);
        telemetry.addData("Logitech webcam", webcamStatus);
        telemetry.addData("Pinpoint X/Y", "%.1f / %.1f in", robotX, robotY);
        telemetry.addData("Pinpoint heading", "%.1f deg", robotHeadingDeg);

        if (bestAllianceTag != null) {
            telemetry.addData("Visible alliance cell", "%s; tag %d; tx %.1f deg",
                    cells[bestAllianceTag.cellIndex].name,
                    bestAllianceTag.result.getFiducialId(),
                    bestAllianceTag.result.getTargetXDegrees());
        } else {
            telemetry.addData("Visible alliance cell", "none in current frame");
        }

        if (blobTarget != null) {
            telemetry.addData("Game element", "%s; tx %.1f deg; area %.2f%%",
                    blobTarget.label, blobTarget.txDegrees, blobTarget.areaPercent);
        }

        for (int i = 0; i < cells.length; i++) {
            CellTrack cell = cells[i];
            String location = cell.hasFieldEstimate
                    ? String.format(Locale.US, "X %.1f, Y %.1f in", cell.fieldXIn, cell.fieldYIn)
                    : "no 3D location estimate yet";
            String positionAge = cell.lastPositionUpdateMs > 0
                    ? String.format(Locale.US, "%.1f s old", (now - cell.lastPositionUpdateMs) / 1000.0)
                    : "never observed";
            String rollInfo = cell.lastRollUpdateMs > 0
                    ? String.format(Locale.US, "%.1f deg (%s)", cell.lastRollDeg,
                    Math.abs(normalizeDegrees(cell.lastRollDeg - Config.CAMERA_TAG_ROLL_OFFSET_DEG))
                            < Config.CELL_UPRIGHT_ROLL_LIMIT_DEG ? "upright" : "inverted")
                    : "unavailable";
            telemetry.addData(cell.name,
                    "%s | %s | tag roll: %s | location: %s (%s)",
                    getCellState(i, now),
                    tagIdRangeForCell(i),
                    rollInfo,
                    location,
                    positionAge);
        }

        telemetry.addLine("A = Limelight shooting alignment; B = Logitech OpenCV blob approach; stick input cancels.");
        telemetry.addLine("Cell state is based on fresh tag observations; OPEN/CLOSED becomes UNKNOWN when stale.");
        telemetry.update();
    }

    private String tagIdRangeForCell(int index) {
        if (index < 0 || index >= Config.CELL_TAG_ID_MIN.length) {
            return "unknown IDs";
        }
        return "IDs " + Config.CELL_TAG_ID_MIN[index] + "-" + Config.CELL_TAG_ID_MAX[index];
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
                // Smooth noisy per-frame vision estimates.
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
            // EasyOpenCV supplies RGBA by default; convert RGBA -> RGB -> HSV.
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