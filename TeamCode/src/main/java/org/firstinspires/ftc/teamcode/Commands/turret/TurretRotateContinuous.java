package org.firstinspires.ftc.teamcode.Commands.turret;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Translation2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Units;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.TurretRotate;

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
        double y = RobotState.getInstance().getEstimatedPose().getX();
        double targetX = -1.82;

        double angle;
        // If on blue team do the first equation set else assume the red team
        if (RobotState.getInstance().getTeam()) {

            double targetY = -1.67;

            //TODO Set the appropriate offest
            angle = getAsin(targetX, targetY, y);
        } else {
            double targetY = -1.67;

            //TODO Set the appropriate offest
            angle = getAsin(targetX, targetY, y);
        }

        double targetPos = angle * TurretRotate.TICKS_PER_DEGREE;

        turretRotate.setTargetPos(targetPos);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launch angle", angle);
        dashboard.sendTelemetryPacket(packet);
    }

    public double getAsin(double targetX, double targetY, double robotY) {
        // Get the distance away from the blue conner
        double h = getHypot(targetX, targetY);
        // Find the distance on x from the blue conner
        double o = Math.abs(targetY - robotY);

        //Find the angle from asin
        return Units.radiansToDegrees(Math.asin(o / h));
    }

    public double getHypot (double x, double y) {
        return RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(new Translation2d(x, y));
    }

    @Override
    public void end(boolean isInterrupted) {
        turretRotate.setTargetPos(0);
    }

}
