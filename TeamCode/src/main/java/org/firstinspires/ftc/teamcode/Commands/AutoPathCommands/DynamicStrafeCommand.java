package org.firstinspires.ftc.teamcode.Commands.AutoPathCommands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.Supplier;

/**
 * A command that generates and executes a Road Runner strafe-to-heading trajectory.
 * <p>
 * The target pose is supplied dynamically at runtime, allowing the same command
 * to be reused for different destinations. This integrates with the FTCLib
 * command-based framework so it can be scheduled in autonomous routines.
 */
public class DynamicStrafeCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final double posTol, headingTol, velTol;
    private final TurnConstraints turnConstraints;
    private final VelConstraint velConstraints;
    private final AccelConstraint accelConstraint;

    private Action trajectoryAction;

    /**
     * Creates a strafe command with drivetrain defaults for constraints and tolerances.
     *
     * @param drivetrain          the drivetrain subsystem
     * @param targetPoseSupplier  supplies the target pose (position + heading)
     */
    public DynamicStrafeCommand(Drivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance;
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;

        addRequirements(drivetrain);
    }

    /**
     * Creates a strafe command with custom motion constraints and default tolerances.
     *
     * @param drivetrain          the drivetrain subsystem
     * @param targetPoseSupplier  supplies the target pose
     * @param turnConstraints     angular velocity/acceleration limits
     * @param velConstraint       linear velocity constraint
     * @param accelConstraint     linear acceleration constraint
     */
    public DynamicStrafeCommand(
            Drivetrain drivetrain,
            Supplier<Pose2d> targetPoseSupplier,
            TurnConstraints turnConstraints,
            VelConstraint velConstraint,
            AccelConstraint accelConstraint
    ) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance;
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;

        addRequirements(drivetrain);
    }

    /**
     * Creates a strafe command with custom tolerances and default constraints.
     *
     * @param drivetrain          the drivetrain subsystem
     * @param targetPoseSupplier  supplies the target pose
     * @param posTol              position tolerance (inches)
     * @param headingTol          heading tolerance (radians)
     * @param velTol              velocity tolerance
     */
    public DynamicStrafeCommand(
            Drivetrain drivetrain,
            Supplier<Pose2d> targetPoseSupplier,
            double posTol,
            double headingTol,
            double velTol
    ) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    /**
     * Creates a strafe command with full custom constraints and tolerances.
     *
     * @param drivetrain          the drivetrain subsystem
     * @param targetPoseSupplier  supplies the target pose
     * @param turnConstraints     angular velocity/acceleration limits
     * @param velConstraint       linear velocity constraint
     * @param accelConstraint     linear acceleration constraint
     * @param posTol              position tolerance (inches)
     * @param headingTol          heading tolerance (radians)
     * @param velTol              velocity tolerance
     */
    public DynamicStrafeCommand(
            Drivetrain drivetrain,
            Supplier<Pose2d> targetPoseSupplier,
            TurnConstraints turnConstraints,
            VelConstraint velConstraint,
            AccelConstraint accelConstraint,
            double posTol,
            double headingTol,
            double velTol
    ) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    /**
     * Builds the trajectory action from the current pose to the supplied target pose.
     */
    @Override
    public void initialize() {
        drivetrain.updatePoseEstimate();
        Pose2d currentPose = drivetrain.localizer.getPose();
        Pose2d targetPose = targetPoseSupplier.get();

        trajectoryAction = drivetrain.actionBuilder(
                        currentPose,
                        turnConstraints, velConstraints, accelConstraint,
                        posTol, headingTol, velTol
                )
                .strafeToLinearHeading(targetPose.position, targetPose.heading)
                .build();
    }

    /**
     * Runs the trajectory action each scheduler cycle.
     */
    @Override
    public void execute() {
        if (trajectoryAction != null) {
            trajectoryAction.run(new TelemetryPacket());
        }
    }

    /**
     * Returns true when the trajectory has completed or cannot run further.
     */
    @Override
    public boolean isFinished() {
        return trajectoryAction == null || !trajectoryAction.run(new TelemetryPacket());
    }

    /**
     * Stops the drivetrain when the command ends, whether finished or interrupted.
     *
     * @param interrupted true if the command was canceled
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}