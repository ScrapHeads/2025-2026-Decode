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
 * A command that generates and follows a spline trajectory to a target position
 * at runtime. The target position is supplied dynamically, so this command can
 * adapt to different goals each time it runs.
 * <p>
 * This wraps Road Runner trajectory building into an FTCLib Command, so it can
 * be scheduled as part of a SequentialCommandGroup or ParallelCommandGroup.
 */
public class DynamicSplineCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final Supplier<Vector2d> targetPositionSupplier;
    private final double targetHeading;
    private final double posTol, headingTol, velTol;
    private final TurnConstraints turnConstraints;
    private final VelConstraint velConstraints;
    private final AccelConstraint accelConstraint;

    private Action trajectoryAction;

    /**
     * Creates a new spline command using the drivetrain's default constraints and tolerances.
     *
     * @param drivetrain the drivetrain subsystem used to move the robot
     * @param targetPositionSupplier supplies the Vector2d target position at runtime
     * @param targetHeading desired final heading at the target (radians)
     */
    public DynamicSplineCommand(Drivetrain drivetrain,
                                Supplier<Vector2d> targetPositionSupplier,
                                double targetHeading) {
        this.drivetrain = drivetrain;
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeading = targetHeading;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance;
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;

        addRequirements(drivetrain);
    }

    /**
     * Creates a new spline command with custom constraints, but default tolerances.
     *
     * @param drivetrain   the drivetrain subsystem
     * @param targetPositionSupplier supplies the target position as a Vector2d
     * @param targetHeading desired final heading at the target (radians)
     * @param turnConstraints maximum angular velocity and acceleration
     * @param velConstraint linear velocity constraint for trajectory
     * @param accelConstraint linear acceleration constraint for trajectory
     */
    public DynamicSplineCommand(Drivetrain drivetrain,
                                Supplier<Vector2d> targetPositionSupplier,
                                double targetHeading,
                                TurnConstraints turnConstraints,
                                VelConstraint velConstraint,
                                AccelConstraint accelConstraint) {
        this.drivetrain = drivetrain;
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeading = targetHeading;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance;
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;

        addRequirements(drivetrain);
    }

    /**
     * Creates a new spline command with custom tolerances, but default constraints.
     * @param drivetrain   the drivetrain subsystem
     * @param targetPositionSupplier supplies the target position as a Vector2d
     * @param targetHeading desired final heading at the target (radians)
     * @param posTol       position tolerance for considering the path complete (inches)
     * @param headingTol   heading tolerance for considering the path complete (radians)
     * @param velTol       velocity tolerance for stopping condition
     */
    public DynamicSplineCommand(Drivetrain drivetrain,
                                Supplier<Vector2d> targetPositionSupplier,
                                double targetHeading,
                                double posTol,
                                double headingTol,
                                double velTol) {
        this.drivetrain = drivetrain;
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeading = targetHeading;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    /**
     * Creates a new spline command with full custom constraints and tolerances.
     *
     * @param drivetrain   the drivetrain subsystem
     * @param targetPositionSupplier supplies the target position as a Vector2d
     * @param targetHeading desired final heading at the target (radians)
     * @param turnConstraints maximum angular velocity and acceleration
     * @param velConstraint linear velocity constraint for trajectory
     * @param accelConstraint linear acceleration constraint for trajectory
     * @param posTol       position tolerance for considering the path complete (inches)
     * @param headingTol   heading tolerance for considering the path complete (radians)
     * @param velTol       velocity tolerance for stopping condition
     */
    public DynamicSplineCommand(Drivetrain drivetrain,
                                Supplier<Vector2d> targetPositionSupplier,
                                double targetHeading,
                                TurnConstraints turnConstraints,
                                VelConstraint velConstraint,
                                AccelConstraint accelConstraint,
                                double posTol,
                                double headingTol,
                                double velTol) {
        this.drivetrain = drivetrain;
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeading = targetHeading;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    /**
     * Builds the trajectory action based on the current pose and target position.
     */
    @Override
    public void initialize() {
        drivetrain.updatePoseEstimate();
        Pose2d currentPose = drivetrain.localizer.getPose();
        Vector2d targetPosition = targetPositionSupplier.get();

        trajectoryAction = drivetrain.actionBuilder(
                        currentPose,
                        turnConstraints, velConstraints, accelConstraint,
                        posTol, headingTol, velTol
                )
                .splineTo(targetPosition, targetHeading)
                .build();
    }

    /**
     * Runs the trajectory action once per scheduler cycle.
     */
    @Override
    public void execute() {
        if (trajectoryAction != null) {
            trajectoryAction.run(new TelemetryPacket());
        }
    }

    /**
     * Ends when the trajectory action finishes or cannot run.
     */
    @Override
    public boolean isFinished() {
        return trajectoryAction == null || !trajectoryAction.run(new TelemetryPacket());
    }

    /**
     * Stops the drivetrain at the end of the command, regardless of whether
     * it was interrupted or completed normally.
     *
     * @param interrupted true if the command was canceled
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
