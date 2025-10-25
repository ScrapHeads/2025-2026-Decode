package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldInches=144
// robotLenIn=18
// robotWidIn=18
import java.util.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;

public final class testPath {
    private testPath() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(56.50, -9.00, new Rotation2d(3.141593)),  // #1  x=56.50in, y=-9.00in, θ=180.0°
    new Pose2d(-9.00, -9.00, new Rotation2d(0.000000))  // #2  x=-9.00in, y=-9.00in, θ=0.0°
);
}
