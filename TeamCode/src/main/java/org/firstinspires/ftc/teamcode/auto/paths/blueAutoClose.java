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
    new Pose2d(-45.00, -39.00, new Rotation2d(2.268929)),  // #2  x=-45.00in, y=-39.00in, θ=130.0°
    new Pose2d(-55.50, -48.00, new Rotation2d(4.066617)),  // #3  x=-55.50in, y=-48.00in, θ=233.0°
    new Pose2d(-12.00, -22.00, new Rotation2d(4.712389)),  // #4  x=-12.00in, y=-22.00in, θ=270.0°
    new Pose2d(-12.00, -50.00, new Rotation2d(4.712389)),  // #5  x=-12.00in, y=-50.00in, θ=270.0°
    new Pose2d(-55.50, -48.00, new Rotation2d(4.066617)),  // #6  x=-55.50in, y=-48.00in, θ=233.0°
    new Pose2d(-3.00, -27.50, new Rotation2d(4.712389)),  // #7  x=-3.00in, y=-27.50in, θ=270.0°
    new Pose2d(13.00, -22.50, new Rotation2d(4.712389)),  // #8  x=13.00in, y=-22.50in, θ=270.0°
    new Pose2d(13.00, -53.50, new Rotation2d(4.712389)),  // #9  x=13.00in, y=-53.50in, θ=270.0°
    new Pose2d(13.00, -44.50, new Rotation2d(4.712389)),  // #10  x=13.00in, y=-44.50in, θ=270.0°
    new Pose2d(-55.50, -48.00, new Rotation2d(4.066617)),  // #11  x=-55.50in, y=-48.00in, θ=233.0°
    new Pose2d(-13.50, -47.50, new Rotation2d(3.926991))  // #12  x=-13.50in, y=-47.50in, θ=225.0°
);
}
