package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Simple Mecanum Drive - Self-Contained Implementation
 *
 * USAGE: Call these methods in your OpMode's game loop (e.g., TeleOp):
 *   SimpleDrive drive = new SimpleDrive();
 *   drive.initMotors(frontLeft, frontRight, backLeft, backRight);
 *
 *   while(opModeIsActive()) {
 *       drive.update(gamepad1);
 *       sleep(20); // Prevent CPU hogging
 *   }
 *
 * - Left stick X/Y: Forward/backward and strafe movement (robot-aligned)
 * - Right stick Y: Rotation (yaw) for turning in place
 *
 * Motors auto-configure direction and mode on init().
 */
public class SimpleDrive {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Deadband to filter small joystick drift
    private static final double DEADBAND = 0.1;

    /**
     * Initialize motors with configuration (direction, brake mode)
     */
    public void initMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;

        // Configure all motors
        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);
    }

    /**
     * Configure a single motor with default settings
     */
    private void configureMotor(DcMotor motor) {
        // Reverse direction as needed (check which wheels move correctly)
        // Uncomment the line below if a wheel moves opposite to expected
        // motor.setDirection(DcMotor.Direction.REVERSE);

        // Set brake mode - BRAKE prevents rolling when stopped
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Disable encoders until needed (or use RUN_TO_POSITION for encoder-based control)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Main update loop - call this every iteration of your game loop
     */
    public void update(Gamepad gamepad) {
        // Stop if any stop command is given (you can add more stop conditions here)
        if (isStopped()) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return;
        }

        // Get joystick values
        double leftX = gamepad.left_stick_x;
        double leftY = -gamepad.left_stick_y;  // Invert Y so forward is positive
        double rightY = gamepad.right_stick_y;

        // Apply deadband to filter small drift
        if (Math.abs(leftX) < DEADBAND) leftX = 0;
        if (Math.abs(leftY) < DEADBAND) leftY = 0;
        if (Math.abs(rightY) < DEADBAND) rightY = 0;

        // Calculate wheel powers for standard mecanum drive
        // LF: forward + strafe - rotate
        // FR: forward - strafe - rotate
        // BL: forward + strafe + rotate
        // BR: forward - strafe + rotate
        double LF = (leftY + leftX - rightY) / 2.0;
        double FR = (leftY - leftX - rightY) / 2.0;
        double BL = (leftY + leftX + rightY) / 2.0;
        double BR = (leftY - leftX + rightY) / 2.0;

        // Apply powers to motors
        frontLeft.setPower(LF);
        frontRight.setPower(FR);
        backLeft.setPower(BL);
        backRight.setPower(BR);
    }

    /**
     * Stops all motors immediately
     */
    public void stop() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    /**
     * Check if user requested stop via Y button
     */
    public boolean isStopped() {
        return false; // Modify to check gamepad.y or other conditions
    }

    /**
     * Swap front/back motor assignments (useful if named incorrectly)
     */
    public void swapFrontBack(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        // Save current motors
        DcMotor tempFL = frontLeft;
        DcMotor tempFR = frontRight;

        // Swap
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;

        // Re-configure swapped motors
        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);
    }
}
