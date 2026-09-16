package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTest", group="Test")
public class MotorTest extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {
        // "motor" must match the name you give this motor in the Driver Station config
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(1.0);
            } else {
                motor.setPower(0.0);
            }

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}