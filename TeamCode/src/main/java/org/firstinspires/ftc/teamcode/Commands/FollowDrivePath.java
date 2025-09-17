package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Command that follows a Road Runner path represented by an {@link Action}.
 * <p>
 * The path is executed incrementally each scheduler cycle until it finishes.
 * Telemetry is sent to the FTC Dashboard for debugging.
 * <p>
 * When the command ends (finished or interrupted), the drivetrain is commanded
 * to stop.
 */
public class FollowDrivePath extends CommandBase {

    private final Action path;
    private final Drivetrain drivetrain;

    private boolean isFinished = false;

    /**
     * Creates a command to follow the given path.
     *
     * @param drivetrain the drivetrain subsystem used to execute motion
     * @param path       the Road Runner action representing the trajectory
     */
    public FollowDrivePath(Drivetrain drivetrain, Action path) {
        this.drivetrain = drivetrain;
        this.path = path;
    }

    /**
     * Runs one iteration of the path and pushes telemetry to the dashboard.
     */
    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        isFinished = path.run(packet);

        // Extra telemetry
        packet.put("test", true);
        packet.put("path", path);

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Stops the drivetrain when the command ends, whether completed or interrupted.
     *
     * @param isInterrupted true if the command was canceled
     */
    @Override
    public void end(boolean isInterrupted) {
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    /**
     * Returns whether the path has finished.
     *
     * Note: Currently returns {@code !isFinished}, which inverts the meaning of
     * the value from {@link Action#run}. If you intend for the command to end
     * when the path is complete, consider returning {@code isFinished} instead.
     */
    @Override
    public boolean isFinished() {
        return !isFinished;  // Check logic: may need to flip to return isFinished
    }
}
