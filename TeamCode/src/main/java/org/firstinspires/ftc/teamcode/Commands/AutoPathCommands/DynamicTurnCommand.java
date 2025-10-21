package org.firstinspires.ftc.teamcode.Commands.AutoPathCommands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

/**
 * A command that generates and executes a Road Runner turn to a target heading.
 * <p>
 * The heading is supplied dynamically using a {@link DoubleSupplier}, so the
 * target can be chosen at runtime rather than being fixed at compile time.
 * This allows more flexible autonomous routines and driver aids.
 */
public class DynamicTurnCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final DoubleSupplier targetHeadingSupplier;
    private final double posTol, headingTol, velTol;
    private final TurnConstraints turnConstraints;
    private final VelConstraint velConstraints;
    private final AccelConstraint accelConstraint;

    private Action trajectoryAction;

    /**
     * Creates a turn command with drivetrain defaults for constraints and tolerances.
     *
     * @param drivetrain             the drivetrain subsystem
     * @param targetHeadingSupplier  supplies the target heading in radians
     */
    public DynamicTurnCommand(Drivetrain drivetrain, DoubleSupplier targetHeadingSupplier) {
        this.drivetrain = drivetrain;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance;
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;

        addRequirements(drivetrain);
    }

    /**
     * Creates a turn command with custom motion constraints but default tolerances.
     *
     * @param drivetrain             the drivetrain subsystem
     * @param targetHeadingSupplier  supplies the target heading in radians
     * @param turnConstraints        angular velocity and acceleration limits
     * @param velConstraint          linear velocity constraint
     * @param accelConstraint        linear acceleration constraint
     */
    public DynamicTurnCommand(
            Drivetrain drivetrain,
            DoubleSupplier targetHeadingSupplier,
            TurnConstraints turnConstraints,
            VelConstraint velConstraint,
            AccelConstraint accelConstraint
    ) {
        this.drivetrain = drivetrain;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance;
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;

        addRequirements(drivetrain);
    }

    /**
     * Creates a turn command with custom tolerances but default constraints.
     *
     * @param drivetrain             the drivetrain subsystem
     * @param targetHeadingSupplier  supplies the target heading in radians
     * @param posTol                 position tolerance (inches)
     * @param headingTol             heading tolerance (radians)
     * @param velTol                 velocity tolerance
     */
    public DynamicTurnCommand(
            Drivetrain drivetrain,
            DoubleSupplier targetHeadingSupplier,
            double posTol,
            double headingTol,
            double velTol
    ) {
        this.drivetrain = drivetrain;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    /**
     * Creates a turn command with full custom constraints and tolerances.
     *
     * @param drivetrain             the drivetrain subsystem
     * @param targetHeadingSupplier  supplies the target heading in radians
     * @param turnConstraints        angular velocity and acceleration limits
     * @param velConstraint          linear velocity constraint
     * @param accelConstraint        linear acceleration constraint
     * @param posTol                 position tolerance (inches)
     * @param headingTol             heading tolerance (radians)
     * @param velTol                 velocity tolerance
     */
    public DynamicTurnCommand(
            Drivetrain drivetrain,
            DoubleSupplier targetHeadingSupplier,
            TurnConstraints turnConstraints,
            VelConstraint velConstraint,
            AccelConstraint accelConstraint,
            double posTol,
            double headingTol,
            double velTol
    ) {
        this.drivetrain = drivetrain;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    /**
     * Builds the Road Runner turn action using the current pose and target heading.
     */
    @Override
    public void initialize() {
        Pose2d currentPose = RobotState.getInstance().getOdometryPose();

        Pose2d targetPose = new Pose2d(
                currentPose.getX(), currentPose.getY(),
                new Rotation2d(targetHeadingSupplier.getAsDouble())
        );

        trajectoryAction = drivetrain.actionBuilder(
                        currentPose,
                        turnConstraints, velConstraints, accelConstraint,
                        posTol, headingTol, velTol
                )
                .turnTo(targetPose.getRotation().getRadians())
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
     * Returns true when the trajectory action is complete or cannot run.
     */
    @Override
    public boolean isFinished() {
        return trajectoryAction == null || !trajectoryAction.run(new TelemetryPacket());
    }

    /**
     * Stops the drivetrain at the end of the command, whether finished or interrupted.
     *
     * @param interrupted true if the command was canceled
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.setDrivePowers(new ChassisSpeeds());
    }
}
