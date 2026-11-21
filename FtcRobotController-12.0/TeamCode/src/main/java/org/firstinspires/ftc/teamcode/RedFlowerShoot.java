
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BIOBUZZ Red Flower", group = "BIOBUZZ")
public class RedFlowerShoot extends LinearOpMode {

    private DcMotor shooter;
    private Servo rotator;

    private static final double SHOOTER_POWER = 0.85;
    private static final double SERVO_REST = 0.0;
    private static final double SERVO_ACTIVE = 0.5;

    private static final long RAMP_UP_MS = 1200;
    private static final long SHOOT_TIME_MS = 4000;

    @Override
    public void runOpMode() {

        // Initialize shooter
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize feeder / rotator servo
        rotator = hardwareMap.get(Servo.class, "servo");

        // Start with servo retracted
        rotator.setPosition(SERVO_REST);

        telemetry.addLine("BIOBUZZ RED FLOWER AUTO");
        telemetry.addLine("Shooter: READY");
        telemetry.addLine("Rotator: REST POSITION");
        telemetry.addLine("DRIVE: DISABLED FOR SAFETY");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            shutdown();
            return;
        }

        // -----------------------------------------
        // STEP 1: RAMP UP SHOOTER
        // -----------------------------------------

        long rampStart = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - rampStart < RAMP_UP_MS) {

            double elapsed = System.currentTimeMillis() - rampStart;

            double rampFraction = elapsed / (double) RAMP_UP_MS;

            double power = SHOOTER_POWER * rampFraction;

            shooter.setPower(power);

            telemetry.addLine("RAMPING UP SHOOTER");
            telemetry.addData("Shooter Power", "%.0f%%", power * 100);
            telemetry.update();

            idle();
        }

        if (!opModeIsActive()) {
            shutdown();
            return;
        }

// Bring shooter to full power
        shooter.setPower(SHOOTER_POWER);

// -----------------------------------------
// STEP 2: WAIT FOR SHOOTER TO STABILIZE
// -----------------------------------------

        telemetry.addLine("SHOOTER AT FULL POWER");
        telemetry.addLine("WAITING BEFORE FEEDING");
        telemetry.update();

        sleep(3000); // Wait 2 seconds before raising rotator

        if (!opModeIsActive()) {
            shutdown();
            return;
        }

// -----------------------------------------
// STEP 3: ACTIVATE ROTATOR
// -----------------------------------------

        rotator.setPosition(SERVO_ACTIVE);

        // -----------------------------------------
        // STEP 3: SHOOT FOR 7 SECONDS
        // -----------------------------------------

        long shootingStart = System.currentTimeMillis();

        while (opModeIsActive()
                && System.currentTimeMillis() - shootingStart < SHOOT_TIME_MS) {

            double remaining =
                    SHOOT_TIME_MS
                            - (System.currentTimeMillis() - shootingStart);

            telemetry.addLine("SHOOTING");
            telemetry.addData("Shooter Power", "85%%");
            telemetry.addData("Time Remaining", "%.1f seconds",
                    remaining / 1000.0);
            telemetry.addData("Rotator", "ACTIVE");
            telemetry.update();

            idle();
        }

        // -----------------------------------------
        // STEP 4: SHUT DOWN
        // -----------------------------------------

        shutdown();

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }

    private void shutdown() {
        if (shooter != null) {
            shooter.setPower(0);
        }

        if (rotator != null) {
            rotator.setPosition(SERVO_REST);
        }
    }
}
