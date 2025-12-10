package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.blueTagPose;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.patters;
import static org.firstinspires.ftc.teamcode.Constants.redTagPose;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Translation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.MathUtil;
import org.firstinspires.ftc.teamcode.RilLib.Math.Units;
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

    public final Limelight3A limelight;

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

    private int i = 0;

    public void setPipeline (int index) {
        TelemetryPacket p = new TelemetryPacket();
        p.put("Called amount", i++);
        p.put("Set index", index);
        dashboard.sendTelemetryPacket(p);
        limelight.pipelineSwitch(index);
    }

    public boolean detectPattern () {
        if (limelight.getStatus().getPipelineIndex() != 1) {
            setPipeline(1);
        }
        LLResult llResult = limelight.getLatestResult();

        for (LLResultTypes.FiducialResult r : llResult.getFiducialResults()) {
            int id = r.getFiducialId();

            if (id >= 21 && id <= 23) {
                RobotState.getInstance().setPattern(patters.get(id));
                return true;
            }
        }
        return false;
    }

    /**
     * Called periodically by the scheduler.
     * <p>
     * Gets the pos of the robot based on limelight mega tag 2
     */
    @Override
    public void periodic() {
        TelemetryPacket p = new TelemetryPacket();
        p.put("LimeLight", limelight.toString());

        Rotation2d curRot = RobotState.getInstance().getOdometryPose().getRotation();
        limelight.updateRobotOrientation(curRot.getDegrees());

        LLResult limeLightResult = limelight.getLatestResult();
        Pose3D robotPose = limeLightResult.getBotpose_MT2();

        p.put("LimeLightResult", limeLightResult.getFiducialResults());
        p.put("LimeLight robot pose", robotPose.toString());
        p.put("Limelight pipeline", limelight.getStatus().getPipelineIndex());
        dashboard.sendTelemetryPacket(p);


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
//        packet.put("Limelight Time", limelightTime);
//        packet.put("Robot time", TimeTracker.getTime());
        packet.put("tag amount", limeLightResult.getFiducialResults().size());
        packet.put("Distance away", distance - 17.78);
        dashboard.sendTelemetryPacket(packet);
    }
}
