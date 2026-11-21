
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;

public class Tuning {

    // Finds the correct names and directions for all four drive motors
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    // Calibrates the goBILDA Pinpoint odometry computer
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }


    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(
                        hardwareMap, Constants.localizerConfig
                ),
                hardwareMap -> new Mecanum(
                        hardwareMap, Constants.drivetrainConfig
                )
        );
    }

}
