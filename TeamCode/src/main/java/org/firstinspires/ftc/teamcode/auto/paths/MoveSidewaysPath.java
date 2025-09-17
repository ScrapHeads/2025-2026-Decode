package org.firstinspires.ftc.teamcode.auto.paths;

// headingWrapHalf=false
// fieldInches=144
// robotLenIn=18
// robotWidIn=18

import com.acmerobotics.roadrunner.Pose2d;

import java.util.Arrays;
import java.util.List;

public final class MoveSidewaysPath {
    private MoveSidewaysPath() {}

    public static final List<Pose2d> PATH = Arrays.asList(
    new Pose2d(-37.00, -9.00, 0.000000),  // #1  x=-37.00in, y=-9.00in, θ=0.0°
    new Pose2d(-37.00, 33.50, 0.000000)  // #2  x=-37.00in, y=33.50in, θ=0.0°
);
}
