package org.firstinspires.ftc.teamcode.Commands.turret;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Translation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Units;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretRotate;
import org.firstinspires.ftc.teamcode.util.ConversionUtil;

public class TurretRotateContinuous extends CommandBase {

    private final TurretRotate turretRotate;

    public TurretRotateContinuous(TurretRotate turretRotate) {
        this.turretRotate = turretRotate;

        addRequirements(turretRotate);
    }

    @Override
    public void execute() {
        //TODO implement the non stop calculation to align with goal

        // Get the robot pose on the x axes
//        double x = RobotState.getInstance().getEstimatedPose().getX();
        double y = RobotState.getInstance().getEstimatedPose().getY();
        double targetX = -1.64;

        double angle;
        // If on blue team do the first equation else assume the red team
        if (RobotState.getInstance().isBlueTeam()) {
            double targetY = -1.64;
//            double targetY = -1.82;

            angle = getAsin(targetX, targetY, y);
        } else {
            double targetY = 1.64;

            angle = getAsin(targetX, targetY, y);
        }

        double targetAngle = (-RobotState.getInstance().getEstimatedPose().getRotation().getDegrees()) - angle - 90;

        double targetPos = ConversionUtil.wrapAngleDeg(targetAngle) * TurretRotate.TICKS_PER_DEGREE;

//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Asin angle", angle);
//        packet.put("Robot angle", (-RobotState.getInstance().getEstimatedPose().getRotation().getDegrees()));
//        packet.put("Launch angle", ConversionUtil.wrapAngleDeg(targetAngle));
//        packet.put("Target pos", targetPos);
//        dashboard.sendTelemetryPacket(packet);

        turretRotate.setTargetPos(targetPos);
    }

    public double getAsin(double targetX, double targetY, double robotY) {
        // Get the distance away from the blue conner
        double h = getHypot(targetX, targetY);
        // Find the distance on x from the blue conner
        double o = targetY - robotY;

        //Find the angle from asin
        return Units.radiansToDegrees(Math.asin(o / h));
    }

    public double getHypot (double x, double y) {
        return RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(new Translation2d(x, y));
    }
}
