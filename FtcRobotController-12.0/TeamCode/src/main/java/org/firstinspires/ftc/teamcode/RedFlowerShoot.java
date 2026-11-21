
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "BIOBUZZ Red Flower", group = "BIOBUZZ")
public class RedFlowerShoot extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private GoBildaPinpointDriver pinpoint;

    private static final double DRIVE_POWER = 0.25;
    private static final double POSITION_TOLERANCE = 1.5;
    private static final double HEADING_TOLERANCE = 3.0;
    private static final double TIMEOUT_SECONDS = 10.0;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "back_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "front_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(
                -1.3420708723894255,
                0.5144078712763749,
                DistanceUnit.INCH);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.recalibrateIMU();
        sleep(500);

        pinpoint.resetPosAndIMU();
        sleep(1000);

        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                7.8515,
                34.6159,
                AngleUnit.DEGREES,
                90.0));

        pinpoint.update();

        telemetry.addLine("BIOBUZZ RED FLOWER");
        telemetry.addLine("Hardware initialized");
        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        boolean reachedFirst = driveToPose(
                19.5508, 34.6371, 0.1037);

        stopDrive();

        telemetry.addData("First position reached", reachedFirst);
        telemetry.update();

    }

    private boolean driveToPose(
            double targetX,
            double targetY,
            double targetHeading) {

        double startTime = getRuntime();

        while (opModeIsActive()) {

            pinpoint.update();

            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosY(DistanceUnit.INCH);
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);

            double dx = targetX - x;
            double dy = targetY - y;
            double distance = Math.hypot(dx, dy);

            double headingError = normalizeAngle(
                    targetHeading - heading);

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading", heading);
            telemetry.addData("Distance remaining", distance);
            telemetry.addData("Heading error", headingError);
            telemetry.update();

            if (distance < POSITION_TOLERANCE
                    && Math.abs(headingError) < HEADING_TOLERANCE) {
                stopDrive();
                return true;
            }

            if (getRuntime() - startTime > TIMEOUT_SECONDS) {
                stopDrive();
                return false;
            }

            double radians = Math.toRadians(heading);

            double forward = dx * Math.cos(radians)
                    + dy * Math.sin(radians);

            double strafe = dx * Math.sin(radians)
                    - dy * Math.cos(radians);

            double forwardPower = clip(forward * 0.025, DRIVE_POWER);
            double strafePower = clip(strafe * 0.025, DRIVE_POWER);
            double turnPower = clip(headingError * 0.012, 0.18);

            double fl = forwardPower + strafePower + turnPower;
            double fr = forwardPower - strafePower - turnPower;
            double bl = forwardPower - strafePower + turnPower;
            double br = forwardPower + strafePower - turnPower;

            double max = Math.max(1.0,
                    Math.max(
                            Math.max(Math.abs(fl), Math.abs(fr)),
                            Math.max(Math.abs(bl), Math.abs(br))));

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            idle();
        }

        stopDrive();
        return false;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
