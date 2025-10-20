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
    new Pose2d(62.76, -8.91, new Rotation2d(3.14)),  // #1  x=62.76in, y=-8.91in, θ=0.0°
    new Pose2d(5.00, -9.00, new Rotation2d(3.14)),  // #2  x=5.00in, y=-9.00in, θ=0.0°
    new Pose2d(-23.64, 23.89, new Rotation2d(3.14)),  // #3  x=-23.64in, y=23.89in, θ=0.0°
    new Pose2d(38.50, 33.50, new Rotation2d(3.14)),  // #4  x=38.50in, y=33.50in, θ=0.0°
    new Pose2d(38.50, -33.00, new Rotation2d(3.14))  // #5  x=38.50in, y=-33.00in, θ=0.0°
);
}
