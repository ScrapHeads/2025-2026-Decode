package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldInches=144
// robotLenIn=18
// robotWidIn=18
import java.util.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;

public final class blueAutoClose {
    private blueAutoClose() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(-59.00, -44.00, new Rotation2d(0.925025)),  // #1  x=-59.00in, y=-44.00in, θ=53.0°
    new Pose2d(-16.00, -15.50, new Rotation2d(2.792527)),  // #2  x=-16.00in, y=-15.50in, θ=160.0°
    new Pose2d(-16.00, -15.50, new Rotation2d(3.926991)),  // #3  x=-16.00in, y=-15.50in, θ=225.0°
    new Pose2d(-12.00, -30.00, new Rotation2d(4.712389)),  // #4  x=-12.00in, y=-30.00in, θ=270.0°
    new Pose2d(-12.00, -55.00, new Rotation2d(4.712389)),  // #5  x=-12.00in, y=-55.00in, θ=270.0°
    new Pose2d(-16.00, -15.50, new Rotation2d(3.926991)),  // #6  x=-16.00in, y=-15.50in, θ=225.0°
    new Pose2d(13.00, -27.00, new Rotation2d(4.712389)),  // #7  x=13.00in, y=-27.00in, θ=270.0°
    new Pose2d(13.00, -52.50, new Rotation2d(4.712389)),  // #8  x=13.00in, y=-52.50in, θ=270.0°
    new Pose2d(-16.00, -15.50, new Rotation2d(4.363323)),  // #9  x=-16.00in, y=-15.50in, θ=250.0°
    new Pose2d(37.50, -27.00, new Rotation2d(4.712389)),  // #10  x=37.50in, y=-27.00in, θ=270.0°
    new Pose2d(37.50, -54.50, new Rotation2d(4.712389))  // #11  x=37.50in, y=-54.50in, θ=270.0°
);
}
