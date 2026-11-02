package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "PinpointTest", group = "Test")
public class PinpointTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // CHANGE THESE to match your robot
        odo.setOffsets(0, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU(); // robot must be sitting still during init

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            telemetry.addData("Device Status", odo.getDeviceStatus());
            telemetry.addData("X (mm)", pos.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", pos.getY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}