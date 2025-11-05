package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=true
// fieldInches=144
// robotLenIn=16
// robotWidIn=18
import java.util.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;

public final class blueAutoFar {
    private blueAutoFar() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(63.00, -15.00, new Rotation2d(-3.141592)),  // #1  x=63.00in, y=-15.00in, θ=-180.0°
    new Pose2d(0.00, -15.00, new Rotation2d(-3.141592)),  // #2  x=0.00in, y=-15.00in, θ=-180.0°
    new Pose2d(49.50, -12.50, new Rotation2d(-2.792527)),  // #3  x=49.50in, y=-12.50in, θ=-160.0°
    new Pose2d(35.50, -28.50, new Rotation2d(-1.570796)),  // #4  x=35.50in, y=-28.50in, θ=-90.0°
    new Pose2d(35.50, -59.50, new Rotation2d(-1.570796)),  // #5  x=35.50in, y=-59.50in, θ=-90.0°
    new Pose2d(24.57, -15.27, new Rotation2d(-2.443460)),  // #6  x=24.57in, y=-15.27in, θ=-140.0°
    new Pose2d(-11.00, -19.00, new Rotation2d(-2.443461)),  // #7  x=-11.00in, y=-19.00in, θ=-140.0°
    new Pose2d(13.00, -28.50, new Rotation2d(-1.570796)),  // #8  x=13.00in, y=-28.50in, θ=-90.0°
    new Pose2d(13.00, -59.00, new Rotation2d(-1.570796)),  // #9  x=13.00in, y=-59.00in, θ=-90.0°
    new Pose2d(-11.00, -19.00, new Rotation2d(-2.443461)),  // #10  x=-11.00in, y=-19.00in, θ=-140.0°
    new Pose2d(-12.38, -30.45, new Rotation2d(-1.570796)),  // #11  x=-12.38in, y=-30.45in, θ=-90.0°
    new Pose2d(-12.50, -53.00, new Rotation2d(-1.570796))  // #12  x=-12.50in, y=-53.00in, θ=-90.0°
);
}
