
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Right Strafe Auto", group = "BIOBUZZ")
public class RightStrafeAuto extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private GoBildaPinpointDriver pinpoint;

    private static final double STRAFE_POWER = 0.35;
    private static final long STRAFE_TIME_MS = 1500;

    // Hardware names from your code
    private static final String FRONT_LEFT_NAME = "front_left";
    private static final String FRONT_RIGHT_NAME = "back_right";
    private static final String BACK_LEFT_NAME = "back_left";
    private static final String BACK_RIGHT_NAME = "front_right";

    // Motor reversal settings from your posted code
    private static final boolean REVERSE_FRONT_LEFT = true;
    private static final boolean REVERSE_FRONT_RIGHT = false;
    private static final boolean REVERSE_BACK_LEFT = true;
    private static final boolean REVERSE_BACK_RIGHT = false;

    private double headingOffsetDeg = 0.0;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Pinpoint
        try {
            pinpoint = hardwareMap.get(
                    GoBildaPinpointDriver.class, "pinpoint");

            pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);

            pinpoint.setEncoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);

            pinpoint.recalibrateIMU();

        } catch (Exception e) {
            pinpoint = null;
            telemetry.addLine("WARNING: Pinpoint not found!");
        }

        telemetry.addLine("Right Strafe Auto Ready");
        telemetry.addLine("Press PLAY to reset heading and strafe right.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        // STEP 1: Capture heading reference like the TeleOp reset button
        if (pinpoint != null) {
            sleep(400);
            pinpoint.update();

            headingOffsetDeg =
                    pinpoint.getHeading(AngleUnit.DEGREES);
        }

        telemetry.addLine("HEADING RESET!");
        telemetry.addData("Heading Offset", headingOffsetDeg);
        telemetry.update();

        // STEP 2: Strafe RIGHT for 1.5 seconds
        long startTime = System.currentTimeMillis();

        try {
            while (opModeIsActive()
                    && System.currentTimeMillis() - startTime < STRAFE_TIME_MS) {

                // Mecanum right strafe
                double fl = STRAFE_POWER;
                double fr = -STRAFE_POWER;
                double bl = -STRAFE_POWER;
                double br = STRAFE_POWER;

                // Apply motor reversals
                if (REVERSE_FRONT_LEFT) fl = -fl;
                if (REVERSE_FRONT_RIGHT) fr = -fr;
                if (REVERSE_BACK_LEFT) bl = -bl;
                if (REVERSE_BACK_RIGHT) br = -br;

                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);

                telemetry.addLine("STRAFING RIGHT");
                telemetry.addData(
                        "Time",
                        System.currentTimeMillis() - startTime);
                telemetry.update();

                idle();
            }
        } finally {
            // Always stop the drive motors
            stopDrive();
        }
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
