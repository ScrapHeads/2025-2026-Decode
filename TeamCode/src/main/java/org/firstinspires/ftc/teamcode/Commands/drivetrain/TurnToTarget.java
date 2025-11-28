package org.firstinspires.ftc.teamcode.Commands.drivetrain;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

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
//        pid.enableContinuousInput(0, 360);

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
        //TODO comment out once done tuning
//        pid.setPID(PARAMS.kp, PARAMS.ki, PARAMS.kd);

        // Get the gamepad speeds from the driver
        double xSpeed = xLimiter.calculate(driver.getLeftY(), TimeTracker.getTime()) * speed;
        double ySpeed = yLimiter.calculate(-driver.getLeftX(), TimeTracker.getTime()) * speed;

        double h;
        double o;
        double angle;
        double asinDegrees;
        double newAsinDegrees;

        // Get the robot pose on the x axes
        double x = RobotState.getInstance().getEstimatedPose().getX();

        // If on red team do the first equation set else assume the blue team
        if (!RobotState.getInstance().getTeam()) {
            // Get the distance away from the red conner
            h = getHypot(-1.7788, 1.8288);
            // Find the distance on x from the red conner
            o = Math.abs(-1.7788 - x);

            //Find the angle from asin
            asinDegrees = Units.radiansToDegrees(Math.asin(o / h));
            
            // Set the appropriate offest
            angle = (asinDegrees + 88.75) * .95;
            newAsinDegrees = 0;

            // Conditions for different asin calculations
            if (x > .5) {
                h = getHypot(-1.8288,1.7788);
                o = Math.abs(-1.8288 - x);
                newAsinDegrees = Units.radiansToDegrees(Math.asin(o / h));
                angle = (newAsinDegrees + 88.75) * 1.01;
            } else if (asinDegrees > 37) {
                h = getHypot(-1.8288,1.7788);
                o = Math.abs(-1.8288 - x);
                newAsinDegrees = Units.radiansToDegrees(Math.asin(o / h));
                angle = (newAsinDegrees + 88.75) * .95;
            }
        } else {
            // Get the distance away from the red conner
            h = getHypot(-1.7788, -1.8288);
            // Find the distance on x from the red conner
            o = Math.abs(-1.7788 - x);

            //Find the angle from asin
            asinDegrees = Units.radiansToDegrees(Math.asin(o / h));

            // Set the appropriate offest
            angle = ((asinDegrees + 88.75) * .95) + 268.75;
            newAsinDegrees = 0;

            // Conditions for different asin calculations
            if (x > .5) {
                h = getHypot(-1.8288,-1.7788);
                o = Math.abs(-1.8288 - x);
                newAsinDegrees = Units.radiansToDegrees(Math.asin(o / h));
                angle = ((newAsinDegrees + 88.75) * 1.01) + 268.75;
            } else if (asinDegrees > 37) {
                h = getHypot(-1.8288,-1.7788);
                o = Math.abs(-1.8288 - x);
                newAsinDegrees = Units.radiansToDegrees(Math.asin(o / h));
                angle = ((newAsinDegrees + 88.75) * .950) + 268.75;
            }
        }



        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launch angle", angle);
        packet.put("Hypot", h);
        packet.put("Opposite", o);
        packet.put("Asin", asinDegrees);
        packet.put("newAsine", newAsinDegrees);
        dashboard.sendTelemetryPacket(packet);

        // Get raw gyro heading (rotation from odometry or IMU)
        Rotation2d rot = RobotState.getInstance().getEstimatedPose().getRotation();

        double rotSpeed = -pid.calculate(angle, rot.getDegrees()); // PID.calculate

        // Apply heading offset
        Rotation2d correctedHeading = rot.minus(RobotState.getInstance().getHeadingOffset());

        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, correctedHeading);

        drivetrain.setDrivePowers(robotSpeeds);
    }

    public double getHypot (double x, double y) {
        return RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(new Translation2d(x, y));
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
