/**
 * OdometryCalibrationTuningProgram.java
 *
 * This program is a dedicated tuning tool designed to calibrate the physical
 * offset of the robot's odometry computer (GoBilda Pinpoint) relative to
 * the robot's true central turning axis. It also calibrates linear and
 * rotational movement factors by comparing motor encoder data against the
 * odometry pod readings.
 *
 * This code should be placed in the 'org.firstinspires.ftc.teamcode' package.
 *
 * Requires:
 * - Access to four motor encoders (FrontLeft, FrontRight, BackLeft, BackRight).
 * - A mechanism to receive odometry pod data (assumed to be provided by a custom Odometry class/listener).
 * - Standard FTC SDK components (MotorController, Telemetry, etc.).
 *
 * NOTE: Actual motor and odometry communication setup (like the specific Odometry class)
 * must be integrated based on the robot's specific hardware setup and GoBilda Pinpoint API.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

// =========================================================================
// STATE MACHINE DEFINITIONS
// =========================================================================

/**
 * Defines the operating states for the calibration program.
 */
enum CalibrationState {
    IDLE,              // Initial state, waiting for start command.
    CALIBRATE_ROTATION,// State for 360-degree spin test.
    CALIBRATE_DISTANCE,// State for linear translation test.
    OFFSET_CALIBRATION,// State for tracking positional offset during maneuvers.
    COMPLETE           // Calibration is finished.
}

/**
 * Represents a simplified Odometry Pod data structure.
 * Replace this with your actual GoBilda Pinpoint data structure.
 */
class OdometryData {
    public double rotationRadians;
    public double xPositionMeters;
    public double yPositionMeters;

    public OdometryData() {
        this.rotationRadians = 0.0;
        this.xPositionMeters = 0.0;
        this.yPositionMeters = 0.0;
    }
}


@TeleOp(name = "Odometry Calibration Tuning Program", group = "Calibration")
public class OdometryCalibrationTuningProgram extends OpMode {

    // =========================================================================
    // HARDWARE AND UTILITY VARIABLES
    // =========================================================================
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    // =========================================================================
    // CALIBRATION STATE VARIABLES
    // =========================================================================
    private CalibrationState currentState = CalibrationState.IDLE;

    // --- Correction Factors ---
    // Rotational: Compares total encoder rotation vs. Odometry rotation (e.g., total_enc_deg / odometry_deg = factor)
    private double rotationalFactor = 1.0;
    // Linear: Compares total encoder distance vs. Odometry distance (e.g., total_enc_in / odometry_in = factor)
    private double linearFactor = 1.0;

    // --- Offset Variables (in mm) ---
    private double offsetX_mm = 0.0;
    private double offsetY_mm = 0.0;

    // --- Odometry & Movement Tracking ---
    private OdometryData currentOdometryData = new OdometryData();
    private double accumulatedEncoderRotation = 0.0; // In Degrees
    private double accumulatedEncoderLinearDistance = 0.0; // In Inches/MM, depending on setup

    // --- Control and Direction Flags ---
    private boolean isForward = true; // General motor direction
    private boolean isPinpointInitialized = false; // Flag to ensure odometry is ready

    // =========================================================================
    // RUN OPMODE
    // =========================================================================

    @Override
    public void init() {

        // 1. Motor Initialization
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        // Set direction modes (assumes standard configuration, will be overridden by calibration)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor behavior
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 2. Telemetry Setup
        telemetry = new Telemetry() {
            @Override
            public Item addData(String caption, String format, Object... args) {
                return null;
            }

            @Override
            public Item addData(String caption, Object value) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, Func<T> valueProducer) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, String format, Func<T> valueProducer) {
                return null;
            }

            @Override
            public boolean removeItem(Item item) {
                return false;
            }

            @Override
            public void clear() {

            }

            @Override
            public void clearAll() {

            }

            @Override
            public Object addAction(Runnable action) {
                return null;
            }

            @Override
            public boolean removeAction(Object token) {
                return false;
            }

            @Override
            public void speak(String text) {

            }

            @Override
            public void speak(String text, String languageCode, String countryCode) {

            }

            @Override
            public boolean update() {
                return false;
            }

            @Override
            public Line addLine() {
                return null;
            }

            @Override
            public Line addLine(String lineCaption) {
                return null;
            }

            @Override
            public boolean removeLine(Line line) {
                return false;
            }

            @Override
            public boolean isAutoClear() {
                return false;
            }

            @Override
            public void setAutoClear(boolean autoClear) {

            }

            @Override
            public int getMsTransmissionInterval() {
                return 0;
            }

            @Override
            public void setMsTransmissionInterval(int msTransmissionInterval) {

            }

            @Override
            public String getItemSeparator() {
                return "";
            }

            @Override
            public void setItemSeparator(String itemSeparator) {

            }

            @Override
            public String getCaptionValueSeparator() {
                return "";
            }

            @Override
            public void setCaptionValueSeparator(String captionValueSeparator) {

            }

            @Override
            public void setDisplayFormat(DisplayFormat displayFormat) {

            }

            @Override
            public Log log() {
                return null;
            }
        };
        telemetry.addData("Calibration State", currentState.name());
        telemetry.addData("Rotational Factor", String.format(Locale.US, "%.3f", rotationalFactor));
        telemetry.addData("Linear Factor", String.format(Locale.US, "%.3f", linearFactor));
        telemetry.addData("Offset X (mm)", String.format(Locale.US, "%.1f", offsetX_mm));
        telemetry.addData("Offset Y (mm)", String.format(Locale.US, "%.1f", offsetY_mm));

        telemetry.update();

        // Initialize state
        currentState = CalibrationState.IDLE;

        // TODO: Initialize odometry data listener here.
        // Example: OdometryManager.getInstance().setUpdateCallback(this::updateOdometry);
    }

    /**
     * Helper method to simulate receiving odometry data updates.
     * In a real application, this would be called by a hardware listener thread.
     * @param data The latest odometry data from the pod.
     */
    public void updateOdometry(OdometryData data) {
        this.currentOdometryData = data;
        isPinpointInitialized = true;
        // Log odometry data to telemetry during calibration if needed
    }

    /**
     * Reads the current motor encoder values and updates the tracking variables.
     * In a real application, this should be called continuously in the loop.
     */
    private void updateEncoderReadings() {
        // Assuming encoder values are retrieved here.
        // For simplicity, we simulate encoder reading, replacing this with actual code.
        double flEnc = frontLeftMotor.getCurrentPosition();
        double frEnc = frontRightMotor.getCurrentPosition();
        double blEnc = backLeftMotor.getCurrentPosition();
        double brEnc = backRightMotor.getCurrentPosition();

        // Simulate accumulated rotation based on differential drive principles
        // Rotation is based on the difference between left and right wheels.
        // NOTE: Wheel circumference/tracks must be factored in here for accurate distance/rotation.
        double rotationDelta = (frEnc - flEnc) / 2.0; // Simplified differential
        double linearDelta = (frEnc + flEnc) / 2.0;   // Simplified linear speed

        accumulatedEncoderRotation += rotationDelta / 100.0; // Conversion to degrees (example)
        accumulatedEncoderLinearDistance += linearDelta / 100.0; // Conversion to target units (example)
    }

    @Override
    public void loop() {

        // 1. Check for Motor/Direction Calibration requests
        if (gamepad1.right_stick_x > 0.5 && currentState == CalibrationState.IDLE) {
            // Toggle general direction
            isForward = !isForward;
            // Apply the direction change (requires re-setting directions on motors)
            frontLeftMotor.setDirection(isForward ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            frontRightMotor.setDirection(isForward ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(isForward ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(isForward ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
            telemetry.addData("Direction", "Switched (Fwd: " + isForward + ")");
            telemetry.update();
        }


        // 2. State Machine Logic
        // Target distance in meters/inches
        double TARGET_DISTANCE_UNIT = 1.0;
        double TARGET_ROTATION_DEGREES = 360.0;
        switch (currentState) {
            case IDLE:
                telemetry.addData("Status", "READY. Press right bumper to start rotation calibration.");
                if (gamepad1.right_bumper) {
                    currentState = CalibrationState.CALIBRATE_ROTATION;
                    // Reset counters for new test
                    accumulatedEncoderRotation = 0.0;
                    accumulatedEncoderLinearDistance = 0.0;
                    // Reset robot position tracking for this test
                    currentOdometryData = new OdometryData();
                }
                break;

            case CALIBRATE_ROTATION:
                // State 2: Rotation Calibration
                telemetry.addData("Status", "ROTATION CALIBRATION: Spin 360 degrees.");
                // Set motor power for spin (e.g., 0.5)
                setMotorPower(0.5);

                // Update position continuously while spinning
                updateEncoderReadings();
                // Simulate Odometry update (In real scenario, this comes from the pod)
                double simulatedOdometryRotation = (accumulatedEncoderRotation / 360.0) * TARGET_ROTATION_DEGREES;
                currentOdometryData.rotationRadians = Math.toRadians(simulatedOdometryRotation);

                // Check completion condition
                if (accumulatedEncoderRotation >= TARGET_ROTATION_DEGREES) {
                    // 2. Calculation and Factor Setting
                    double actualEncoderRotation = accumulatedEncoderRotation;
                    double actualOdometryRotation = Math.toDegrees(currentOdometryData.rotationRadians);

                    // Calculate the rotational factor
                    // Factor = Actual Encoder Movement / Actual Odometry Reading
                    if (actualOdometryRotation > 0) {
                        rotationalFactor = actualEncoderRotation / actualOdometryRotation;
                    }

                    telemetry.addData("Encoder Rotation", String.format(Locale.US, "%.2f deg", actualEncoderRotation));
                    telemetry.addData("Odometry Rotation", String.format(Locale.US, "%.2f deg", actualOdometryRotation));
                    telemetry.addData("Calculated Rotational Factor", String.format(Locale.US, "%.3f", rotationalFactor));

                    // Transition
                    currentState = CalibrationState.CALIBRATE_DISTANCE;
                    telemetry.addData("Status", "ROTATION COMPLETE. Press B to start distance calibration.");
                    // Stop motors
                    setMotorPower(0.0);
                }
                break;

            case CALIBRATE_DISTANCE:
                // State 3: Distance Calibration
                telemetry.addData("Status", "DISTANCE CALIBRATION: Drive 1 unit forward.");
                // Set motor power for forward drive (e.g., 0.6)
                setMotorPower(0.6);

                // Update position continuously while driving
                updateEncoderReadings();
                // Simulate Odometry update (In real scenario, this comes from the pod)
                currentOdometryData.xPositionMeters = (accumulatedEncoderLinearDistance / TARGET_DISTANCE_UNIT) * (1.0 / linearFactor); // Simple linear movement along X

                // Check completion condition
                if (accumulatedEncoderLinearDistance >= TARGET_DISTANCE_UNIT) {
                    // 3. Calculation and Factor Setting
                    double actualEncoderDistance = accumulatedEncoderLinearDistance;
                    double actualOdometryDistance = Math.abs(currentOdometryData.xPositionMeters);

                    // Calculate the linear factor
                    // Factor = Actual Encoder Movement / Actual Odometry Reading
                    if (actualOdometryDistance > 0) {
                        linearFactor = actualEncoderDistance / actualOdometryDistance;
                    }

                    telemetry.addData("Encoder Distance", String.format(Locale.US, "%.2f units", actualEncoderDistance));
                    telemetry.addData("Odometry Distance", String.format(Locale.US, "%.2f units", actualOdometryDistance));
                    telemetry.addData("Calculated Linear Factor", String.format(Locale.US, "%.3f", linearFactor));

                    // Transition
                    currentState = CalibrationState.OFFSET_CALIBRATION;
                    telemetry.addData("Status", "DISTANCE COMPLETE. Now performing OFFSET calibration. Press B.");
                    // Stop motors
                    setMotorPower(0.0);
                }
                break;

            case OFFSET_CALIBRATION:
                // State 4: Offset Calibration (Simulated)
                // This state should guide the user to perform a specific maneuver (e.g., straight drive, circle)
                // while logging the position discrepancy.
                telemetry.addData("Status", "OFFSET CALIBRATION: Drive in a known pattern (e.g., straight line).");
                telemetry.addData("Instruction", "Manually drive a straight line and log the difference between expected and observed odometry.");

                // Example logic for tracking offset (requires continuous, real-time odometry/encoder data)
                // X_offset = (Actual_X_Encoder - Expected_X_Odometry) * Rotational_Factor
                // Y_offset = (Actual_Y_Encoder - Expected_Y_Odometry) * Rotational_Factor
                // Simplified example: assuming the encoder system is perfect for finding the offset bias
                // This logic would be far more complex in a real system.
                offsetX_mm = (currentOdometryData.xPositionMeters - accumulatedEncoderLinearDistance) * 100.0; // Dummy calculation in mm
                offsetY_mm = 0.0; // Assume Y offset is 0 for simplicity in this example.

                // Allow user to confirm offset calculation
                if (gamepad1.right_bumper) {
                    telemetry.addData("Status", "OFFSET MANEUVER COMPLETE.");
                    currentState = CalibrationState.COMPLETE;
                    telemetry.addData("Status", "CALIBRATION COMPLETE. Press START to reset.");
                }
                break;

            case COMPLETE:
                // Final state
                telemetry.addData("Status", "READY FOR NEXT RUN. Press START to reset.");
                if (gamepad1.start) {
                    // Reset all calibration data
                    rotationalFactor = 1.0;
                    linearFactor = 1.0;
                    offsetX_mm = 0.0;
                    offsetY_mm = 0.0;
                    currentState = CalibrationState.IDLE;
                    telemetry.addData("Status", "RESET TO IDLE.");
                }
                break;
        }

        // 5. Telemetry Update
        if (isPinpointInitialized) {
            telemetry.addData("Odometry X (m)", String.format(Locale.US, "%.3f", currentOdometryData.xPositionMeters));
            telemetry.addData("Odometry Y (m)", String.format(Locale.US, "%.3f", currentOdometryData.yPositionMeters));
            telemetry.addData("Encoder Total Rot (deg)", String.format(Locale.US, "%.2f", accumulatedEncoderRotation));
            telemetry.update();
        }
    }

    /**
     * Sets the power for all drive motors.
     * @param power The desired power (-1.0 to 1.0).
     */
    private void setMotorPower(double power) {
        frontLeftMotor.setPower(power * (isForward ? 1.0 : -1.0));
        frontRightMotor.setPower(power * (isForward ? 1.0 : -1.0));
        backLeftMotor.setPower(power * (isForward ? 1.0 : -1.0));
        backRightMotor.setPower(power * (isForward ? 1.0 : -1.0));
    }
}