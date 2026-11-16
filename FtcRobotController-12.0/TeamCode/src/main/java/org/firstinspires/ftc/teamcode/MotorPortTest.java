package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Motor Port Test - runs TWO motors at the same time, each with its own controls.
 *
 * Shows every motor in the active Robot Configuration (so you can see if a name is wrong).
 * The motor power ramps up smoothly. Change RAMP_SECONDS (and RAMP_DOWN_TOO) in the code below.
 *
 *  1) TRIGGER motor: the motor on port TRIGGER_PORT (3). Only the left trigger drives it.
 *  2) BUTTON motor:  the motor on port BUTTON_PORT. If BUTTON_PORT is -1, it is the first other motor in the
 *                    configuration. Driven by the left stick and the A / X / B / Y buttons.
 *
 * Both run at the same time. Nothing to switch.
 *
 * Controls (Gamepad 1):
 *  - Left trigger : TRIGGER motor (port 3) at LEFT_TRIGGER_POWER (+40%)
 *  - Left stick Y : BUTTON motor (up = +, down = -)
 *  - A            : BUTTON motor at -100%
 *  - X            : BUTTON motor at -75%
 *  - B            : BUTTON motor at -50%
 *  - Y            : BUTTON motor at -25%
 */
@TeleOp(name = "Motor Port Test", group = "Test")
public class MotorPortTest extends LinearOpMode {

    // ===== ADJUST THESE IN THE CODE =====

    /** The left trigger always spins the motor on this port. */
    private static final int TRIGGER_PORT = 3;

    /** The stick and buttons spin the motor on this port. -1 = automatically use the first other motor. */
    private static final int BUTTON_PORT = -1;

    /** Ramp time: seconds to go from 0 to full power. Bigger = slower ramp. 0 = no ramp. */
    private static final double RAMP_SECONDS = 1.0;

    /** true = the motors also ease down when you let go. false = they stop right away. */
    private static final boolean RAMP_DOWN_TOO = false;

    /** Power for the left trigger (40%, positive). Use -0.40 to spin the other way. */
    private static final double LEFT_TRIGGER_POWER = 0.40;

    /** Moves 'current' toward 'requested'. Speeding up (or reversing) is ramped; slowing down is instant unless RAMP_DOWN_TOO. */
    private double ramp(double requested, double current, double dt) {
        boolean speedingUp = Math.abs(requested) > Math.abs(current) || requested * current < 0;
        if (RAMP_SECONDS <= 0.0 || !(speedingUp || RAMP_DOWN_TOO)) {
            return requested;
        }
        double maxStep = dt / RAMP_SECONDS; // full power takes RAMP_SECONDS
        double change = requested - current;
        return current + Math.max(-maxStep, Math.min(maxStep, change));
    }

    @Override
    public void runOpMode() {
        // Find every motor in the active configuration
        List<String> names = new ArrayList<>(hardwareMap.getAllNames(DcMotor.class));
        Collections.sort(names);

        List<DcMotor> motors = new ArrayList<>();
        for (String name : names) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
            motors.add(motor);
        }

        VoltageSensor battery = null;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            battery = sensor;
            break;
        }

        // the trigger motor = the one on TRIGGER_PORT
        int triggerIndex = -1;
        for (int i = 0; i < motors.size(); i++) {
            if (motors.get(i).getPortNumber() == TRIGGER_PORT) {
                triggerIndex = i;
                break;
            }
        }

        // the button motor = the one on BUTTON_PORT, or else the first motor that is not the trigger motor
        int buttonIndex = -1;
        for (int i = 0; i < motors.size(); i++) {
            if (i == triggerIndex) continue;
            if (BUTTON_PORT < 0 || motors.get(i).getPortNumber() == BUTTON_PORT) {
                buttonIndex = i;
                break;
            }
        }

        double buttonPower = 0.0;   // power actually sent to the button motor (after the ramp)
        double triggerPower = 0.0;  // power actually sent to the trigger motor (after the ramp)

        // ----- INIT: show what was found -----
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("=== MOTOR PORT TEST ===");
            telemetry.addData("Battery", battery != null ? String.format("%.2f V", battery.getVoltage()) : "no sensor");
            if (names.isEmpty()) {
                telemetry.addLine("NO MOTORS in the active configuration!");
                telemetry.addLine("Check Configure Robot: add a motor, save it, and make it active.");
            } else {
                telemetry.addLine("Motors found in the configuration:");
                for (int i = 0; i < names.size(); i++) {
                    String role = i == triggerIndex ? "  <- LEFT TRIGGER motor"
                            : (i == buttonIndex ? "  <- STICK / BUTTON motor" : "");
                    telemetry.addData("  " + (i + 1), "%s (port %d)%s", names.get(i), motors.get(i).getPortNumber(), role);
                }
                if (triggerIndex < 0) {
                    telemetry.addLine("NO MOTOR on port " + TRIGGER_PORT + ": the left trigger will do nothing.");
                    telemetry.addLine("Configure Robot: add a motor on port " + TRIGGER_PORT + ", save, make it active.");
                }
                if (buttonIndex < 0) {
                    telemetry.addLine("No second motor found for the stick / buttons. Add another motor to the configuration"
                            + (BUTTON_PORT >= 0 ? " on port " + BUTTON_PORT : "") + ".");
                }
                telemetry.addLine("Press PLAY. Both motors can run at the same time.");
            }
            telemetry.update();
            sleep(50);
        }

        // ----- RUN -----
        ElapsedTime timer = new ElapsedTime();
        double lastTime = timer.seconds();

        while (opModeIsActive()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            lastTime = now;

            if (motors.isEmpty()) {
                telemetry.addLine("NO MOTORS in the active configuration!");
                telemetry.update();
                idle();
                continue;
            }

            // power you ask for: the button motor
            double buttonRequested = Math.abs(gamepad1.left_stick_y) > 0.08 ? -gamepad1.left_stick_y : 0.0;
            if (gamepad1.a) buttonRequested = -1.0;
            if (gamepad1.b) buttonRequested = -0.5;
            if (gamepad1.x) buttonRequested = -0.75;
            if (gamepad1.y) buttonRequested = -0.25;

            // power you ask for: the trigger motor
            double triggerRequested = gamepad1.left_trigger > 0.1 ? LEFT_TRIGGER_POWER : 0.0;

            buttonPower = ramp(buttonRequested, buttonPower, dt);
            triggerPower = ramp(triggerRequested, triggerPower, dt);

            // each motor gets only its own power, and both run together
            for (int i = 0; i < motors.size(); i++) {
                if (i == triggerIndex) motors.get(i).setPower(triggerPower);
                else if (i == buttonIndex) motors.get(i).setPower(buttonPower);
                else motors.get(i).setPower(0.0);
            }

            telemetry.addLine("=== MOTOR PORT TEST ===");
            telemetry.addData("Battery", battery != null ? String.format("%.2f V", battery.getVoltage()) : "no sensor");
            telemetry.addData("Ramp time (0 to full)", RAMP_SECONDS <= 0.0 ? "OFF" : String.format("%.1f s", RAMP_SECONDS));

            telemetry.addLine("--- LEFT TRIGGER motor ---");
            if (triggerIndex >= 0) {
                telemetry.addData("Motor", "%s (port %d)", names.get(triggerIndex), TRIGGER_PORT);
                telemetry.addData("Power (requested / actual)", "%.2f / %.2f", triggerRequested, triggerPower);
                telemetry.addData("Encoder position", motors.get(triggerIndex).getCurrentPosition());
            } else {
                telemetry.addLine("No motor on port " + TRIGGER_PORT);
            }

            telemetry.addLine("--- STICK / BUTTON motor ---");
            if (buttonIndex >= 0) {
                telemetry.addData("Motor", "%s (port %d)", names.get(buttonIndex), motors.get(buttonIndex).getPortNumber());
                telemetry.addData("Power (requested / actual)", "%.2f / %.2f", buttonRequested, buttonPower);
                telemetry.addData("Encoder position", motors.get(buttonIndex).getCurrentPosition());
            } else {
                telemetry.addLine("No second motor found");
            }

            telemetry.addLine("Left trigger = port " + TRIGGER_PORT + " at +40%");
            telemetry.addLine("Stick = button motor | A = -100% | X = -75% | B = -50% | Y = -25%");
            telemetry.update();
        }

        for (DcMotor motor : motors) motor.setPower(0.0);
    }
}