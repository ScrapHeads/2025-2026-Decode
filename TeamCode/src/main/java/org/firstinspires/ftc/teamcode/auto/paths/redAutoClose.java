package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldInches=144
// robotLenIn=16
// robotWidIn=18
import java.util.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;

public final class redAutoClose {
    private redAutoClose() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(-59.50, 45.00, new Rotation2d(5.358160)),  // #1  x=-59.50in, y=45.00in, θ=307.0°
    new Pose2d(-29.50, 24.00, new Rotation2d(3.665190)),  // #2  x=-29.50in, y=24.00in, θ=210.0°
    new Pose2d(-18.50, 16.00, new Rotation2d(2.356194)),  // #3  x=-18.50in, y=16.00in, θ=135.0°
    new Pose2d(-14.00, 25.00, new Rotation2d(1.570796)),  // #4  x=-14.00in, y=25.00in, θ=90.0°
    new Pose2d(-14.00, 51.00, new Rotation2d(1.570796)),  // #5  x=-14.00in, y=51.00in, θ=90.0°
    new Pose2d(-18.50, 16.00, new Rotation2d(2.356194)),  // #6  x=-18.50in, y=16.00in, θ=135.0°
    new Pose2d(-3.00, 24.50, new Rotation2d(1.570796)),  // #7  x=-3.00in, y=24.50in, θ=90.0°
    new Pose2d(13.00, 22.50, new Rotation2d(1.570796)),  // #8  x=13.00in, y=22.50in, θ=90.0°
    new Pose2d(13.00, 59.50, new Rotation2d(1.570796)),  // #9  x=13.00in, y=59.50in, θ=90.0°
    new Pose2d(13.00, 44.50, new Rotation2d(1.570796)),  // #10  x=13.00in, y=44.50in, θ=90.0°
    new Pose2d(-18.50, 16.00, new Rotation2d(2.356194)),  // #11  x=-18.50in, y=16.00in, θ=135.0°
    new Pose2d(36.50, 15.00, new Rotation2d(1.570796))  // #12  x=36.50in, y=15.00in, θ=90.0°
);
}
