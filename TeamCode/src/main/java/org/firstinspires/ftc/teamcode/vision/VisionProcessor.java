//package org.firstinspires.ftc.teamcode.vision;
//
//import static org.firstinspires.ftc.teamcode.Constants.DISTANCE_CONSTANT;
//import static org.firstinspires.ftc.teamcode.Constants.LEFT_SCALING;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Rotation2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//
///**
// * Processes HuskyLens vision data to estimate robot pose.
// * <p>
// * This class provides utility methods to convert detected HuskyLens blocks
// * (representing tags or objects) into approximate robot-relative poses.
// * It accounts for camera mounting offsets, field of view, and robot motion
// * latency when used alongside Road Runner localization.
// */
//public class VisionProcessor {
//
//    private double cameraForwardOffset;
//    private double cameraLeftOffset;
//    private double cameraHeight;
//    private double cameraHeadingOffset;
//
//    // Calibration constants (must be tuned on the actual robot)
//    private double distanceConstant = DISTANCE_CONSTANT;
//    private double leftScaling = LEFT_SCALING;
//
//    private static final int IMAGE_WIDTH = Constants.IMAGE_WIDTH;
//    private static final double CAMERA_HORIZONTAL_FOV = Constants.CAMERA_HORIZONTAL_FOV;
//
//    /**
//     * Creates a VisionProcessor with explicit camera mounting parameters.
//     *
//     * @param forward  forward offset of the camera from the robot center (inches)
//     * @param left     lateral offset of the camera from the robot center (inches)
//     * @param height   vertical offset of the camera from the ground (inches)
//     * @param heading  yaw offset of the camera relative to the robot heading (radians)
//     */
//    public VisionProcessor(double forward, double left, double height, double heading) {
//        this.cameraForwardOffset = forward;
//        this.cameraLeftOffset = left;
//        this.cameraHeight = height;
//        this.cameraHeadingOffset = heading;
//    }
//
//    /**
//     * Creates a VisionProcessor using a {@link CameraParams} struct.
//     *
//     * @param params camera offset parameters (forward, lateral, vertical, yaw)
//     */
//    public VisionProcessor(CameraParams params) {
//        this(params.forwardOffset, params.lateralOffset, params.verticalOffset, params.yawOffset);
//    }
//
//    /**
//     * Converts a HuskyLens block into an approximate robot-relative pose.
//     * <p>
//     * This uses the block size to estimate forward distance, the block x-position
//     * to estimate lateral offset, and derives heading based on horizontal offset
//     * relative to the camera's field of view. Camera mounting offsets are applied.
//     *
//     * @param block HuskyLens block containing detection data
//     * @return a Pose2d estimate of the detected object relative to the robot,
//     *         or null if input is invalid
//     */
//    public Pose2d blockToRobotPose(HuskyLens.Block block) {
//        if (block == null) return null;
//
//        // 1) Estimate forward distance (+X) from tag size
//        double avgSize = (block.width + block.height) / 2.0;
//        if (avgSize <= 0) return null; // avoid div by zero
//        double tagForward = distanceConstant / avgSize;
//
//        // 2) Estimate lateral offset (+Y) from horizontal pixel position
//        double normalizedX = (block.x - (IMAGE_WIDTH / 2.0)) / (IMAGE_WIDTH / 2.0);
//        double tagLeft = normalizedX * leftScaling;
//
//        Vector2d visionPos = new Vector2d(tagForward, tagLeft);
//
//        // 3) Estimate heading (yaw) from horizontal offset
//        double yawRadians = normalizedX * (CAMERA_HORIZONTAL_FOV / 2.0);
//        Rotation2d visionHeading = new Rotation2d(Math.cos(yawRadians), Math.sin(yawRadians));
//
//        // 4) Apply camera mount offset
//        Vector2d offset = new Vector2d(cameraForwardOffset, cameraLeftOffset);
//        Rotation2d finalHeading = visionHeading.plus(cameraHeadingOffset);
//
//        return new Pose2d(visionPos.plus(offset), finalHeading);
//    }
//
//    /**
//     * Produces a corrected robot pose using HuskyLens vision,
//     * compensating for robot motion during image capture.
//     * <p>
//     * This method compares the drivetrain's estimated pose before and after
//     * the image was processed, then adjusts the vision-based estimate to
//     * account for any movement that occurred during that interval.
//     *
//     * @param drivetrain drivetrain used for localization
//     * @param block      HuskyLens block containing detection data
//     * @return a corrected Pose2d of the robot, or null if vision data is invalid
//     */
//    public Pose2d getCorrectedRobotPoseFromTag(Drivetrain drivetrain, HuskyLens.Block block) {
////        if (block == null) return null;
////
////        drivetrain.updatePoseEstimate();
////        Pose2d poseBefore = drivetrain.localizer.getPose();
////
////        Pose2d visionPose = blockToRobotPose(block);
////        if (visionPose == null) return null;
////
////        drivetrain.updatePoseEstimate();
////        Pose2d poseNow = drivetrain.localizer.getPose();
////
////        Pose2d motionOffset = poseNow.minusExp(poseBefore);
////
////        return new Pose2d(
////                visionPose.position.plus(motionOffset.position),
////                visionPose.heading.plus(motionOffset.heading.toDouble())
////        );
//        return null;
//    }
//}