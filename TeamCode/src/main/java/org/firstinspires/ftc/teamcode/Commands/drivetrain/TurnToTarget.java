package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RilLib.Control.PID.PIDController;
import org.firstinspires.ftc.teamcode.RilLib.Math.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Translation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.RilLib.Math.Units;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.TimeTracker;

@Config
public class TurnToTarget extends CommandBase {

    public static class Params {
        public double kp = 0.027;
        public double ki = 0.0;
        public double kd = 0.0;
    }
    public static Params PARAMS = new Params();

    private final Drivetrain drivetrain;
    private final GamepadEx driver;
    private final double speed;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final PIDController pid = new PIDController(PARAMS.kp, PARAMS.ki, PARAMS.kd);

    public TurnToTarget (Drivetrain drivetrain, GamepadEx driver, double speed, Vision vision) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.driver = driver;

        pid.setTolerance(1);

        vision.setPipeline(0);
        pid.enableContinuousInput(-180, 180);

        xLimiter = new SlewRateLimiter(2, TimeTracker.getTime());
        yLimiter = new SlewRateLimiter(2, TimeTracker.getTime());

        addRequirements(drivetrain);
        addRequirements(vision);
    }

    @Override
    public void initialize() {

    }

    /**
     * Reads joystick input every cycle and applies it as a drive power.
     */
    @Override
    public void execute() {
        //TODO comment out once done tuning
//        pid.setPID(PARAMS.kp, PARAMS.ki, PARAMS.kd);

        // Get the gamepad speeds from the driver
        double xSpeed = xLimiter.calculate(driver.getLeftY(), TimeTracker.getTime()) * speed;
        double ySpeed = yLimiter.calculate(-driver.getLeftX(), TimeTracker.getTime()) * speed;

        // Get the robot pose on the x axes
        double x = RobotState.getInstance().getEstimatedPose().getX();

        double angle;
        // If on red team do the first equation set else assume the blue team
        if (!RobotState.getInstance().getTeam()) {
            double targetX = -1.7788;
            double targetY = 1.8288;

            double asinDegrees = getAsin(targetX, targetY, x);
            
            // Set the appropriate offest
            angle = (asinDegrees + 88.75) * .95;

            // Conditions for different asin calculations
            if (x > .5) {
                angle = (getAsin(-targetY, -targetX, x) + 88.75) * 1.01;
            } else if (asinDegrees > 37) {
                angle = (getAsin(-targetY, -targetX, x) + 88.75) * .95;
            }
        } else {
            double targetX = -1.7788;
            double targetY = -1.8288;

            double asinDegrees = getAsin(targetX, targetY, x);

            // Set the appropriate offest
            angle = ((-asinDegrees - 103) * .935);

            // Conditions for different asin calculations
            if (x > .5) {
                angle = ((-getAsin(targetY, targetX, x) - 89) * 1.03);
            } else if (asinDegrees > 40) {
                angle = ((-getAsin(targetY, targetX, x) - 91.5) * 1.03);
            }
        }



        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launch angle", angle);
        dashboard.sendTelemetryPacket(packet);

        // Get raw gyro heading (rotation from odometry or IMU)
        Rotation2d rot = RobotState.getInstance().getEstimatedPose().getRotation();

        double rotSpeed = -pid.calculate(angle, rot.getDegrees()); // PID.calculate

        // Apply heading offset
        Rotation2d correctedHeading = rot.minus(RobotState.getInstance().getHeadingOffset());

        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, correctedHeading);

        drivetrain.setDrivePowers(robotSpeeds);
    }

    public double getAsin(double targetX, double targetY, double robotX) {
        // Get the distance away from the blue conner
        double h = getHypot(targetX, targetY);
        // Find the distance on x from the blue conner
        double o = Math.abs(targetX - robotX);

        //Find the angle from asin
        return Units.radiansToDegrees(Math.asin(o / h));
    }

    public double getHypot (double x, double y) {
        return RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(new Translation2d(x, y));
    }

    @Override
    public boolean isFinished() {

        return driver.isDown(GamepadKeys.Button.A) || driver.isDown(GamepadKeys.Button.B) || driver.isDown(GamepadKeys.Button.Y) || driver.isDown(GamepadKeys.Button.DPAD_RIGHT);
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
