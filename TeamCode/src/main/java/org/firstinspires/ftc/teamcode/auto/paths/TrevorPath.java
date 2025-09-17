package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldInches=144
// robotLenIn=18
// robotWidIn=18

import com.acmerobotics.roadrunner.Pose2d;

import java.util.Arrays;
import java.util.List;

public final class TrevorPath {
    private TrevorPath() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(12.00, -62.50, 0.000000),  // #1  x=12.00in, y=-62.50in, θ=0.0°
    new Pose2d(57.50, -33.00, 4.712389),  // #2  x=57.50in, y=-33.00in, θ=270.0°
    new Pose2d(15.00, 14.50, 5.585054),  // #3  x=15.00in, y=14.50in, θ=320.0°
    new Pose2d(39.26, 37.59, 2.705260),  // #4  x=39.26in, y=37.59in, θ=155.0°
    new Pose2d(-9.45, 57.17, 2.879794),  // #5  x=-9.45in, y=57.17in, θ=165.0°
    new Pose2d(-39.06, 8.96, 4.014257),  // #6  x=-39.06in, y=8.96in, θ=230.0°
    new Pose2d(-35.88, -62.76, 1.570796)  // #7  x=-35.88in, y=-62.76in, θ=90.0°
);
}
