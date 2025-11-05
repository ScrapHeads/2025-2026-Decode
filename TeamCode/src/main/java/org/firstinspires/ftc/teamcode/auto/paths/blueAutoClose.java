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
    new Pose2d(-16.00, -15.50, new Rotation2d(2.792527)),  // #2  x=-16.00in, y=-15.50in, θ=160.0°
    new Pose2d(-11.00, -19.00, new Rotation2d(3.839724)),  // #3  x=-11.00in, y=-19.00in, θ=220.0°
    new Pose2d(-12.00, -28.50, new Rotation2d(4.712389)),  // #4  x=-12.00in, y=-28.50in, θ=270.0°
    new Pose2d(-12.00, -53.00, new Rotation2d(4.712389)),  // #5  x=-12.00in, y=-53.00in, θ=270.0°
    new Pose2d(-11.00, -19.00, new Rotation2d(3.839724)),  // #6  x=-11.00in, y=-19.00in, θ=220.0°
    new Pose2d(13.00, -28.50, new Rotation2d(4.712389)),  // #7  x=13.00in, y=-28.50in, θ=270.0°
    new Pose2d(13.00, -60.50, new Rotation2d(4.712389)),  // #8  x=13.00in, y=-60.50in, θ=270.0°
    new Pose2d(-11.00, -19.00, new Rotation2d(3.839724)),  // #9  x=-11.00in, y=-19.00in, θ=220.0°
    new Pose2d(35.50, -28.50, new Rotation2d(4.712389)),  // #10  x=35.50in, y=-28.50in, θ=270.0°
    new Pose2d(35.50, -59.50, new Rotation2d(4.712389))  // #11  x=35.50in, y=-59.50in, θ=270.0°
);
}
