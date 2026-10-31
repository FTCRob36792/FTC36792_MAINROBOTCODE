package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Simple Mecanum Drive implementation for FTC robots.
 * - Left stick X/Y controls forward/backward and strafe movement
 * - Right stick Y controls rotation (yaw)
 */
public class SimpleDrive {

    // Deadband to filter small joystick movements
    private static final double DEADBAND = 0.1;

    /**
     * Calculates wheel powers for standard mecanum drive:
     * P_LF = forward - strafe - rotate
     * P_LR = forward + strafe - rotate
     * P_RF = forward - strafe + rotate
     * P_RR = forward + strafe + rotate
     */
    public void calculateAndSetPowers(
            DcMotor leftFront, DcMotor leftRear,
            DcMotor rightFront, DcMotor rightRear,
            Gamepad gamepad) {

        // Get joystick values
        double leftX = gamepad.left_stick_x;
        double leftY = -gamepad.left_stick_y;  // Invert Y so forward is positive
        double rightY = gamepad.right_stick_y;

        // Apply deadband
        if (Math.abs(leftX) < DEADBAND) leftX = 0;
        if (Math.abs(leftY) < DEADBAND) leftY = 0;
        if (Math.abs(rightY) < DEADBAND) rightY = 0;

        // Calculate wheel powers for standard mecanum drive configuration
        double LF = (leftY - leftX - rightY) / 2.0;
        double LR = (leftY + leftX - rightY) / 2.0;
        double RF = (leftY - leftX + rightY) / 2.0;
        double RR = (leftY + leftX + rightY) / 2.0;

        // Apply powers to motors
        leftFront.setPower(LF);
        leftRear.setPower(LR);
        rightFront.setPower(RF);
        rightRear.setPower(RR);
    }

    /**
     * Calculates wheel powers with reversed motor directions.
     * Use this if your robot moves opposite to expected direction.
     */
    public void calculateAndSetPowersReversed(
            DcMotor leftFront, DcMotor leftRear,
            DcMotor rightFront, DcMotor rightRear,
            Gamepad gamepad) {

        double leftX = gamepad.left_stick_x;
        double leftY = -gamepad.left_stick_y;
        double rightY = gamepad.right_stick_y;

        // Apply deadband
        if (Math.abs(leftX) < DEADBAND) leftX = 0;
        if (Math.abs(leftY) < DEADBAND) leftY = 0;
        if (Math.abs(rightY) < DEADBAND) rightY = 0;

        // Reversed calculation
        double LF = (-leftY - leftX + rightY) / 2.0;
        double LR = (-leftY + leftX + rightY) / 2.0;
        double RF = (-leftY - leftX - rightY) / 2.0;
        double RR = (-leftY + leftX - rightY) / 2.0;

        leftFront.setPower(LF);
        leftRear.setPower(LR);
        rightFront.setPower(RF);
        rightRear.setPower(RR);
    }

    /**
     * Stops all motors
     */
    public void stopMotors(
            DcMotor leftFront, DcMotor leftRear,
            DcMotor rightFront, DcMotor rightRear) {

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
