package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RilLib.Math.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

/**
 * A continuous drive command for TeleOp.
 * <p>
 * This command reads joystick input from a {@link GamepadEx} and converts it
 * into a {@link PoseVelocity2d}, which is then applied to the drivetrain. It
 * remains scheduled for as long as TeleOp runs, providing field- or robot-centric
 * driving depending on how the drivetrain interprets the velocity.
 * <p>
 * When the command ends (either because the OpMode stops or it is interrupted),
 * the drivetrain is commanded to stop.
 */
public class DriveContinous extends CommandBase {

    private final Drivetrain drivetrain;
    private final GamepadEx driver;
    private final double speed;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotLimiter;

    /**
     * Creates a new continuous drive command.
     *
     * @param drivetrain the drivetrain subsystem to control
     * @param driver     the gamepad used for driver input
     * @param speed      scaling factor for input values (0.0–1.0 typical)
     */
    public DriveContinous(Drivetrain drivetrain, GamepadEx driver, double speed) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.speed = speed;

        xLimiter = new SlewRateLimiter(10, TimeTracker.getTime());
        yLimiter = new SlewRateLimiter(10, TimeTracker.getTime());
        rotLimiter = new SlewRateLimiter(10, TimeTracker.getTime());

        if (RobotState.getInstance().getTeam()) {
            // Blue team: field forward at -90 degrees
            RobotState.getInstance().setHeadingOffset(Rotation2d.fromDegrees(-90));
        } else {
            // Red team: field forward at +90 degrees
            RobotState.getInstance().setHeadingOffset(Rotation2d.fromDegrees(90));
        }

        addRequirements(drivetrain);
    }

    /**
     * Reads joystick input every cycle and applies it as a drive power.
     */
    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(driver.getLeftY(), TimeTracker.getTime()) * speed;
        double ySpeed = yLimiter.calculate(-driver.getLeftX(), TimeTracker.getTime()) * speed;
        double rotSpeed = rotLimiter.calculate(-driver.getRightX(), TimeTracker.getTime()) * speed;

        // Get raw gyro heading (rotation from odometry or IMU)
        Rotation2d rawHeading = RobotState.getInstance().getOdometryPose().getRotation();

        // Apply heading offset
        Rotation2d correctedHeading = rawHeading.minus(RobotState.getInstance().getHeadingOffset());

        // Field-centric transform using corrected heading
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, correctedHeading);

//        double xSpeed = xLimiter.calculate(driver.getLeftY(), TimeTracker.getTime()) * speed;
//        double ySpeed = yLimiter.calculate(-driver.getLeftX(), TimeTracker.getTime()) * speed;
//        double rotSpeed = rotLimiter.calculate(-driver.getRightX(), TimeTracker.getTime()) * speed;
//
//        Rotation2d rot = RobotState.getInstance().getOdometryPose().getRotation();
//        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, rot);

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
