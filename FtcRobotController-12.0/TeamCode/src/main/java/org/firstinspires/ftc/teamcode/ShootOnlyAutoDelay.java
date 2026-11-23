
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BASE.Config;

@Autonomous(name = "Shoot Only Auto Delay", group = "Competition")
public class ShootOnlyAutoDelay extends LinearOpMode {

    private static final double SHOOTER_VELOCITY = 1850;
    private static final String SHOOTER_NAME = "shooter";

    private static final double DRIVE_POWER = 0.50;
    private static final long GATE_DELAY_MS = 15000;
    private static final long SHOOT_DURATION_MS = 5000;
    private static final long DRIVE_DURATION_MS = 4000;

    @Override
    public void runOpMode() {

        DcMotorEx shooter =
                hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        Servo customServo =
                hardwareMap.get(Servo.class, Config.SERVO_NAME);

        DcMotor frontLeft =
                hardwareMap.get(DcMotor.class, Config.FRONT_LEFT_NAME);
        DcMotor frontRight =
                hardwareMap.get(DcMotor.class, Config.FRONT_RIGHT_NAME);
        DcMotor backLeft =
                hardwareMap.get(DcMotor.class, Config.BACK_LEFT_NAME);
        DcMotor backRight =
                hardwareMap.get(DcMotor.class, Config.BACK_RIGHT_NAME);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(0);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize gate at Position 1
        double targetServoAngleDeg = Config.SERVO_ANGLE_POS1_DEG;
        customServo.setPosition(0.0);

        telemetry.addLine("Shoot and Drive Auto Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        try {
            // Start shooter immediately
            shooter.setVelocity(SHOOTER_VELOCITY);

            // Wait 7 seconds before raising gate
            long startTime = System.currentTimeMillis();

            while (opModeIsActive() &&
                    System.currentTimeMillis() - startTime < GATE_DELAY_MS) {
                telemetry.addLine("Waiting to raise gate");
                telemetry.update();
                idle();
            }

            if (!opModeIsActive()) return;

            // Raise gate to Position 2
            targetServoAngleDeg = Config.SERVO_ANGLE_POS2_DEG;

            double range = Math.max(
                    0.001,
                    Config.SERVO_ANGLE_MAX_DEG -
                            Config.SERVO_ANGLE_MIN_DEG
            );

            double normPos =
                    (targetServoAngleDeg -
                            Config.SERVO_ANGLE_MIN_DEG) / range;

            normPos = Math.max(0.0, Math.min(1.0, normPos));

            if (Config.REVERSE_SERVO) {
                normPos = 1.0 - normPos;
            }

            customServo.setPosition(normPos);

            // Keep shooting for another 5 seconds
            long shootStart = System.currentTimeMillis();

            while (opModeIsActive() &&
                    System.currentTimeMillis() - shootStart < SHOOT_DURATION_MS) {
                telemetry.addData("Shooter Velocity", shooter.getVelocity());
                telemetry.addLine("Shooting");
                telemetry.update();
                idle();
            }

            // Stop shooter after shooting
            shooter.setVelocity(0);
            shooter.setPower(0);

            if (!opModeIsActive()) return;

            // Drive forward at 50% power for 4 seconds
            frontLeft.setPower(DRIVE_POWER);
            frontRight.setPower(DRIVE_POWER);
            backLeft.setPower(DRIVE_POWER);
            backRight.setPower(DRIVE_POWER);

            long driveStart = System.currentTimeMillis();

            while (opModeIsActive() &&
                    System.currentTimeMillis() - driveStart < DRIVE_DURATION_MS) {
                telemetry.addLine("Driving Forward");
                telemetry.addData("Drive Power", DRIVE_POWER);
                telemetry.update();
                idle();
            }

        } finally {
            // Stop everything
            shooter.setVelocity(0);
            shooter.setPower(0);

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}
