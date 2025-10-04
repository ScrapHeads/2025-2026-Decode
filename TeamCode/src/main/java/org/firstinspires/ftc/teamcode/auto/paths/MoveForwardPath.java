package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldInches=144
// robotLenIn=18
// robotWidIn=18
import java.util.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;

public final class MoveForwardPath {
    private MoveForwardPath() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(-61.00, -10.00, new Rotation2d(0.000000)),  // #1  x=-61.00in, y=-10.00in, θ=0.0°
    new Pose2d(-9.50, -9.50, new Rotation2d(0.000000))  // #2  x=-9.50in, y=-9.50in, θ=0.0°
);
}
