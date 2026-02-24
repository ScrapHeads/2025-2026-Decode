package org.firstinspires.ftc.teamcode.state;

import static org.firstinspires.ftc.teamcode.Constants.CENTER_TO_FRONT_OFFSET_CM;
import static org.firstinspires.ftc.teamcode.Constants.blueTagPose;
import static org.firstinspires.ftc.teamcode.Constants.redTagPose;

import org.firstinspires.ftc.teamcode.RilLib.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation.InterpolatingTreeMap;
import org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation.InverseInterpolator;
import org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation.TurretStateInterpolator;

public class TurretLookupTable {

    // Distance -> ShooterState(rpm, hoodDeg)
    private final InterpolatingTreeMap<Double, TurretState> table =
            new InterpolatingTreeMap<>(
                    InverseInterpolator.forDouble(),
                    new TurretStateInterpolator()
            );

    public TurretLookupTable() {
        // Put your tuned points here.
        // Make sure distance units are consistent (inches OR meters, etc.)
        table.put(70.0, new TurretState(4200.0, 744.0));
        table.put(80.0, new TurretState(4375.0, 1000.0));
        table.put(90.0, new TurretState(4450.0, 1100.0));
        table.put(100.0, new TurretState(4550.0, 1250));
        table.put(110.0, new TurretState(4600.0, 1300));
        table.put(120.0, new TurretState(4650.0, 1500.0));
        table.put(130.0, new TurretState(4850.0, 1600.0));
        table.put(140.0, new TurretState(4925.0, 1650.0));
        table.put(150.0, new TurretState(5050.0, 1700.0));
        table.put(160.0, new TurretState(5100.0, 1750));
        table.put(170.0, new TurretState(5200.0, 1850));
        table.put(180.0, new TurretState(5300.0, 1850.0));
        table.put(190.0, new TurretState(5350.0, 1950.0));
        table.put(200.0, new TurretState(5500.0, 2000.0));
        table.put(210.0, new TurretState(5700.0, 2000.0));
        table.put(220.0, new TurretState(5800.0, 2100.0));
        table.put(230.0, new TurretState(5900.0, 2100.0));
        table.put(240.0, new TurretState(6000.0, 2150.0));
        table.put(250.0, new TurretState(6050.0, 2200.0));
        table.put(260.0, new TurretState(6175.0, 2200.0));
        table.put(270.0, new TurretState(6225.0, 2200.0));
        table.put(280.0, new TurretState(6325.0, 2200.0));
        table.put(290.0, new TurretState(6400.0, 2250.0));
        table.put(300.0, new TurretState(6475.0, 2300.0));
        table.put(310.0, new TurretState(6525.0, 2300.0));
    }

    public TurretState get(double distance) {
        TurretState state = table.get(distance);
        if (state == null) {
            // Only happens if table is empty, but this keeps you safe.
            return new TurretState(0.0, 0.0);
        }
        return state;
    }

    public double getDistance () {
        Pose2d tagLocation = RobotState.getInstance().getTeam() ? blueTagPose : redTagPose;
        double distance = 100 * RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(tagLocation.getTranslation());
        return distance - CENTER_TO_FRONT_OFFSET_CM;
    }

    public void clear() {
        table.clear();
    }
}
