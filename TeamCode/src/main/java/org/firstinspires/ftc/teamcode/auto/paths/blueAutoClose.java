package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldInches=144
// robotLenIn=16
// robotWidIn=18
import java.util.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;

public final class blueAutoClose {
    private blueAutoClose() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(-59.50, -45.00, new Rotation2d(0.925025)),  // #1  x=-59.50in, y=-45.00in, θ=53.0°
    new Pose2d(-29.50, -24.00, new Rotation2d(2.617995)),  // #2  x=-29.50in, y=-24.00in, θ=150.0°
    new Pose2d(-18.50, -16.00, new Rotation2d(3.839724)),  // #3  x=-18.50in, y=-16.00in, θ=220.0°
    new Pose2d(-14.00, -25.00, new Rotation2d(4.712389)),  // #4  x=-14.00in, y=-25.00in, θ=270.0°
    new Pose2d(-14.00, -51.00, new Rotation2d(4.712389)),  // #5  x=-14.00in, y=-51.00in, θ=270.0°
    new Pose2d(-18.50, -16.00, new Rotation2d(3.839724)),  // #6  x=-18.50in, y=-16.00in, θ=220.0°
    new Pose2d(-3.00, -27.50, new Rotation2d(4.712389)),  // #7  x=-3.00in, y=-27.50in, θ=270.0°
    new Pose2d(13.00, -22.50, new Rotation2d(4.712389)),  // #8  x=13.00in, y=-22.50in, θ=270.0°
    new Pose2d(13.00, -59.50, new Rotation2d(4.712389)),  // #9  x=13.00in, y=-59.50in, θ=270.0°
    new Pose2d(13.00, -44.50, new Rotation2d(4.712389)),  // #10  x=13.00in, y=-44.50in, θ=270.0°
    new Pose2d(-18.50, -16.00, new Rotation2d(3.839724)),  // #11  x=-18.50in, y=-16.00in, θ=220.0°
    new Pose2d(37.50, -16.50, new Rotation2d(4.712389))  // #12  x=37.50in, y=-16.50in, θ=270.0°
);
}
