
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

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
}
