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
    new Pose2d(-33.00, -26.50, new Rotation2d(2.530728)),  // #2  x=-33.00in, y=-26.50in, θ=145.0°
    new Pose2d(-55.50, -48.00, new Rotation2d(4.066617)),  // #3  x=-55.50in, y=-48.00in, θ=233.0°
    new Pose2d(-14.00, -17.00, new Rotation2d(4.712389)),  // #4  x=-14.00in, y=-17.00in, θ=270.0°
    new Pose2d(-12.00, -50.00, new Rotation2d(4.712389)),  // #5  x=-12.00in, y=-50.00in, θ=270.0°
    new Pose2d(-55.50, -48.00, new Rotation2d(4.066617)),  // #6  x=-55.50in, y=-48.00in, θ=233.0°
    new Pose2d(13.00, -22.50, new Rotation2d(4.712389)),  // #7  x=13.00in, y=-22.50in, θ=270.0°
    new Pose2d(13.00, -53.50, new Rotation2d(4.712389)),  // #8  x=13.00in, y=-53.50in, θ=270.0°
    new Pose2d(-55.50, -48.00, new Rotation2d(4.066617)),  // #9  x=-55.50in, y=-48.00in, θ=233.0°
    new Pose2d(0.00, -47.50, new Rotation2d(3.141593))  // #10  x=0.00in, y=-47.50in, θ=180.0°
);
}
