package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.blueTagPose;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.redTagPose;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.VecBuilder;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

/**
 * Vision subsystem for HuskyLens integration.
 * <p>
 * This subsystem manages:
 * <ul>
 *   <li>Communication with the HuskyLens sensor.</li>
 *   <li>Conversion of detected blocks into corrected robot poses using.</li>
 *   <li>Rate-limited periodic updates to prevent overloading the I2C bus.</li>
 *   <li>Exposing the latest corrected pose to other commands and subsystems.</li>
 * </ul>
 */
public class Vision implements Subsystem {

    private final Limelight3A limelight;

    /**
     * Creates a Vision subsystem with HuskyLens and camera calibration parameters.
     *
     */
    public Vision(HardwareMap hm) {
        limelight = hm.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        startLimelight();
    }

    public void startLimelight () {limelight.start();}
    public void stopLimeLight () {limelight.stop();}


    public void setPipeline (int index) {
        limelight.pipelineSwitch(index);
    }

//    public Pose2d getPose () {
//        LLResult limeLightResult = limelight.getLatestResult();
//        Pose3D robotPose = limeLightResult.getBotpose_MT2();
//
//        Pose2d pose = new Pose2d(
//                robotPose.getPosition().x,
//                robotPose.getPosition().y,
//                new Rotation2d(robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));
//
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Robot vision pose", pose.toString());
//        packet.put("Limelight Time", limeLightResult.getTimestamp());
//        packet.put("Robot time", TimeTracker.getTime());
//        dashboard.sendTelemetryPacket(packet);
//
//        return pose;
//    }



    /**
     * Called periodically by the scheduler.
     * <p>
     * Performs a rate-limited HuskyLens read, filters blocks by tracked tag IDs
     * and minimum pixel size, and computes corrected poses using the vision processor.
     * Updates {@code latestPose} if a valid correction is available.
     */
    @Override
    public void periodic() {
        Rotation2d curRot = RobotState.getInstance().getOdometryPose().getRotation();
        limelight.updateRobotOrientation(curRot.getDegrees());

        LLResult limeLightResult = limelight.getLatestResult();
        Pose3D robotPose = limeLightResult.getBotpose_MT2();

        Pose2d pose = new Pose2d(
                robotPose.getPosition().x,
                robotPose.getPosition().y,
                new Rotation2d(robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));

        double limelightTime = TimeTracker.convertTime(limeLightResult.getControlHubTimeStamp() / 1000.0);

        if (pose.getX() == 0 && pose.getY() == 0) return;
//        ChassisSpeeds robotVel = RobotState.getInstance().getChassisSpeeds();

        RobotState.getInstance().addVisionObservation(pose, limelightTime,
                VecBuilder.fill(
                        Math.pow(0.8, limeLightResult.getFiducialResults().size()) * (limeLightResult.getBotposeAvgDist()) * 2,
                        Math.pow(0.8, limeLightResult.getFiducialResults().size()) * (limeLightResult.getBotposeAvgDist()) * 2,
                        9999999));

        Pose2d tagLocation = RobotState.getInstance().getTeam() ? blueTagPose : redTagPose;
        double distance = 100 * RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(tagLocation.getTranslation());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Robot vision pose", pose.toString());
        packet.put("Estimated Pose", RobotState.getInstance().getEstimatedPose());
        packet.put("Limelight Time", limelightTime);
        packet.put("Robot time", TimeTracker.getTime());
        packet.put("tag amount", limeLightResult.getFiducialResults().size());
        packet.put("Distance away", distance - 17.78);
        dashboard.sendTelemetryPacket(packet);
    }
}
