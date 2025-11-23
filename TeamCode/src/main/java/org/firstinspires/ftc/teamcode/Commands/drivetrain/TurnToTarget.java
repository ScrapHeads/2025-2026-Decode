package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RilLib.Control.PID.PIDController;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

public class TurnToTarget extends CommandBase {
    private final Pose2d targetPose;
    private final Drivetrain drivetrain;
    private final GamepadEx driver;
    private final double speed;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final PIDController pid = new PIDController(Drivetrain.PARAMS.kS, Drivetrain.PARAMS.kV, Drivetrain.PARAMS.kA);

    public TurnToTarget (Drivetrain drivetrain, GamepadEx driver, Pose2d targetPose, double speed) {
        this.targetPose = targetPose;
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.driver = driver;

        xLimiter = new SlewRateLimiter(2, TimeTracker.getTime());
        yLimiter = new SlewRateLimiter(2, TimeTracker.getTime());

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    /**
     * Reads joystick input every cycle and applies it as a drive power.
     */
    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(driver.getLeftY(), TimeTracker.getTime()) * speed;
        double ySpeed = yLimiter.calculate(-driver.getLeftX(), TimeTracker.getTime()) * speed;

        Pose2d curPose = RobotState.getInstance().getEstimatedPose();
        Rotation2d rotTarget = curPose.minus(targetPose).getRotation();
        double distance = curPose.getTranslation().getDistance(targetPose.getTranslation());

        rotTarget.rotateBy(new Rotation2d(distance * 0));

        double rotSpeed = 0; // PID.calculate

        Rotation2d rot = RobotState.getInstance().getEstimatedPose().getRotation();
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, rot);

        drivetrain.setDrivePowers(robotSpeeds);
    }

    /**
     * Stops the drivetrain when the command ends, whether completed or interrupted.
     *
     * @param isInterrupted true if the command was canceled or OpMode ended
     */
    @Override
    public void end(boolean isInterrupted) {
        drivetrain.setDrivePowers(new ChassisSpeeds());
    }
}
