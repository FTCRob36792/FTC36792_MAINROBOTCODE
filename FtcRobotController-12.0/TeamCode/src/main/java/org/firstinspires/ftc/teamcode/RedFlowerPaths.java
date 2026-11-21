
package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class RedFlowerPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(7.8515, 34.6159, 90);
    private final Pose redFlowerStart = poseFactory.of(19.5508, 34.6371, 0.1037);
    private final Pose redFlowerStart2 = poseFactory.of(57.2122, 16.7404, 45);
    private final Pose redFlowerShoot = poseFactory.of(57.2825, 5.043, 90);

    public Path redFlowerStart() {
        return Paths.line(start, redFlowerStart).tangent();
    }

    public Path redFlowerStart2() {
        return Paths.line(redFlowerStart, redFlowerStart2).constant(redFlowerStart2);
    }

    public Path redFlowerShoot() {
        return Paths.line(redFlowerStart2, redFlowerShoot).constant(redFlowerShoot);
    }
}
