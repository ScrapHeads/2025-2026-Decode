package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Units;

public class ConversionUtil {
    public static Pose2d convertPose2D(com.acmerobotics.roadrunner.Pose2d pose) {
        return new Pose2d(
                Units.inchesToMeters(pose.position.x),
                Units.inchesToMeters(pose.position.y),
                new Rotation2d(pose.heading.toDouble()));
    }

    public static com.acmerobotics.roadrunner.Pose2d convertPose2D(Pose2d pose) {
        Vector2d vec = new Vector2d(
                Units.metersToInches(pose.getX()),
                Units.metersToInches(pose.getY()));
        return new com.acmerobotics.roadrunner.Pose2d(vec, pose.getRotation().getRadians());
    }

    public static ChassisSpeeds convertChassisSpeeds(PoseVelocity2d speeds) {
        return new ChassisSpeeds(
                Units.inchesToMeters(speeds.linearVel.x),
                Units.inchesToMeters(speeds.linearVel.y),
                speeds.angVel);
    }

    public static PoseVelocity2d convertChassisSpeeds(ChassisSpeeds speeds) {
        Vector2d vec = new Vector2d(
                Units.metersToInches(speeds.vxMetersPerSecond),
                Units.metersToInches(speeds.vyMetersPerSecond));
        return new PoseVelocity2d(vec, speeds.omegaRadiansPerSecond);
    }
}
