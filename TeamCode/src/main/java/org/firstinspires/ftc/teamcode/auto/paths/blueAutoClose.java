package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldSize=3.66
// robotLen=0.4572
// robotWid=0.4064
// measurementUnit=m
import java.util.*;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;

public final class blueAutoClose {
    private blueAutoClose() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(-1.068, -1.434, new Rotation2d(1.570796)),  // #1  x=-1.068m, y=-1.434m, θ=90.0°
    new Pose2d(-0.483, -0.513, new Rotation2d(-2.478368)),  // #2  x=-0.483m, y=-0.513m, θ=223.0° 
    new Pose2d(-0.307, -0.710, new Rotation2d(4.712389)),  // #3  x=-0.307m, y=-0.710m, θ=270.0°
    new Pose2d(-0.313, -1.382, new Rotation2d(4.712389)),  // #4  x=-0.313m, y=-1.382m, θ=270.0°
    new Pose2d(-0.483, -0.513, new Rotation2d(-2.478368)),  // #5  x=-0.483m, y=-0.513m, θ=223.0°
    new Pose2d(0.343, -0.710, new Rotation2d(4.712389)),  // #6  x=0.343m, y=-0.710m, θ=270.0°
    new Pose2d(0.349, -1.398, new Rotation2d(4.712389)),  // #7  x=0.349m, y=-1.398m, θ=270.0°
    new Pose2d(0.343, -1.073, new Rotation2d(4.712389)),  // #8  x=0.343m, y=-1.073m, θ=270.0°
    new Pose2d(-0.478, -0.513, new Rotation2d(-2.478368)),  // #9  x=-0.478m, y=-0.513m, θ=223.0°
    new Pose2d(0.919, -0.710, new Rotation2d(4.712389))  // #10  x=0.919m, y=-0.710m, θ=270.0°
);
}
