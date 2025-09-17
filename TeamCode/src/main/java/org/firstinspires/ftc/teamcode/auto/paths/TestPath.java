package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldInches=144
// robotLenIn=18
// robotWidIn=18

import com.acmerobotics.roadrunner.Pose2d;

import java.util.Arrays;
import java.util.List;

public final class TestPath {
    private TestPath() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(11.85, -62.76, 0.000000),  // #1  x=11.85in, y=-62.76in, θ=0.0°
    new Pose2d(42.93, -7.44, 0.000000),  // #2  x=42.93in, y=-7.44in, θ=0.0°
    new Pose2d(9.64, 45.92, 2.356194),  // #3  x=9.64in, y=45.92in, θ=135.0°
    new Pose2d(-49.59, 35.64, 3.403392),  // #4  x=-49.59in, y=35.64in, θ=195.0°
    new Pose2d(-4.80, -6.71, 3.490659),  // #5  x=-4.80in, y=-6.71in, θ=200.0°
    new Pose2d(-35.64, -62.51, 3.141593)  // #6  x=-35.64in, y=-62.51in, θ=180.0°
);
}
