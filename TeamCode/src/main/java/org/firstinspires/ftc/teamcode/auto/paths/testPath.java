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
    new Pose2d(15, 15, new Rotation2d(3.14)),  // #1  x=62.76in, y=-8.91in, θ=0.0°
    new Pose2d(0, 0, new Rotation2d(0))  // #2  x=5.00in, y=-9.00in, θ=0.0°
);
}
