package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.AutoAlignConfig;

/**
 * Command that automatically aligns the robot to a HuskyLens-detected tag.
 * <p>
 * - If the driver provides stick input, driver control takes priority.
 * - If no tag is seen, the robot stops.
 * - If a tag is seen but not aligned, proportional control is applied using
 *   gains and tolerances from {@link AutoAlignConfig}.
 * - When aligned, the drivetrain is commanded to hold position.
 * <p>
 * Telemetry is published both to the Driver Hub and the FTC Dashboard.
 */
public class VisionAutoAlign extends CommandBase {

    private final Vision vision;
    private final Drivetrain drivetrain;
    private final GamepadEx driver;

    /**
     * Creates a new VisionAutoAlign command.
     *
     * @param vision     the Vision subsystem for HuskyLens integration
     * @param drivetrain the drivetrain subsystem used to apply drive powers
     * @param driver     the driver gamepad (used to override auto-align if active)
     */
    public VisionAutoAlign(Vision vision, Drivetrain drivetrain, GamepadEx driver) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.driver = driver;

        addRequirements(vision, drivetrain);
    }

    /**
     * Runs every scheduler cycle:
     * <ul>
     *   <li>Checks driver input - if active, driver control overrides auto-align.</li>
     *   <li>Reads HuskyLens blocks - if no valid tag, stops the robot.</li>
     *   <li>Finds the first valid tracked tag - checks size threshold.</li>
     *   <li>Computes pixel error vs target alignment point.</li>
     *   <li>Applies proportional strafe/forward/turn correction if not within tolerance.</li>
     * </ul>
     */
    @Override
    public void execute() {
        HuskyLens.Block[] blocks = vision.detectObject();

        // Driver input
        double driverForward = -driver.getLeftY();
        double driverStrafe  = driver.getLeftX();
        double driverTurn    = driver.getRightX();

        // If driver is moving, skip auto-align
        boolean driverActive = Math.abs(driverForward) > 0.05 ||
                Math.abs(driverStrafe) > 0.05 ||
                Math.abs(driverTurn) > 0.05;

        if (driverActive) {
            drivetrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(driverForward, driverStrafe),
                    driverTurn
            ));
            sendTelemetry(false, false, false);
            return;
        }

        // If no tags, stop
        if (blocks.length == 0) {
            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sendTelemetry(false, false, false);
            return;
        }

        HuskyLens.Block block = blocks[0];
        AutoAlignConfig config = Constants.TAG_CONFIGS.get(block.id);

        // If tag not configured or too small, stop
        if (config == null || Math.min(block.width, block.height) < config.minWidth) {
            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sendTelemetry(true, false, false);
            return;
        }

        // Pixel error relative to desired target position
        double errorX = block.x - config.targetX;
        double errorY = block.y - config.targetY;

        // Check tolerance
        boolean aligned = Math.abs(errorX) < config.toleranceX &&
                Math.abs(errorY) < config.toleranceY;

        if (aligned) {
            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sendTelemetry(true, false, true);
            return;
        }

        // Proportional correction
        double strafeCmd  = errorX * config.kP_strafe;
        double forwardCmd = (config.targetY - block.y) * config.kP_forward;
        double turnCmd    = errorX * config.kP_heading;

        drivetrain.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardCmd, strafeCmd),
                turnCmd
        ));

        sendTelemetry(true, true, false);
    }

    /**
     * Sends both Driver Hub telemetry and FTC Dashboard telemetry.
     *
     * @param tagSeen   true if a tag is detected
     * @param aligning  true if robot is actively aligning
     * @param aligned   true if robot is within alignment tolerance
     */
    private void sendTelemetry(boolean tagSeen, boolean aligning, boolean aligned) {
        // Driver Hub
        Constants.tele.addData("AutoAlign", Constants.AUTO_ALIGN_ENABLED ? "ON" : "OFF");
        Constants.tele.addData("Tag Detected", tagSeen ? "YES" : "NO");
        Constants.tele.addData("Aligning", aligning ? "YES" : "NO");
        Constants.tele.addData("Aligned", aligned ? "YES" : "NO");
        Constants.tele.update();

        // Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("AutoAlign", Constants.AUTO_ALIGN_ENABLED ? "ON" : "OFF");
        packet.put("Tag Detected", tagSeen ? "YES" : "NO");
        packet.put("Aligning", aligning ? "YES" : "NO");
        packet.put("Aligned", aligned ? "YES" : "NO");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /**
     * Runs until explicitly canceled (never finishes on its own).
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Stops the drivetrain when the command ends.
     *
     * @param interrupted true if canceled/interrupted
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}