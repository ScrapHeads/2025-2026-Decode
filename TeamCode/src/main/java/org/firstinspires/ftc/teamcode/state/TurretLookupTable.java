package org.firstinspires.ftc.teamcode.state;

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
        table.put(80.0, new TurretState(0.0, 0.0));
        table.put(90.0, new TurretState(0.0, 0.0));
        table.put(100.0, new TurretState(0.0, 0.0));
        table.put(110.0, new TurretState(0.0, 0.0));
        table.put(120.0, new TurretState(0.0, 0.0));
        table.put(130.0, new TurretState(0.0, 0.0));
        table.put(140.0, new TurretState(0.0, 0.0));
        table.put(150.0, new TurretState(0.0, 0.0));
        table.put(160.0, new TurretState(0.0, 0.0));
        table.put(170.0, new TurretState(0.0, 0.0));
        table.put(180.0, new TurretState(0.0, 0.0));
        table.put(190.0, new TurretState(0.0, 0.0));
        table.put(200.0, new TurretState(0.0, 0.0));
        table.put(210.0, new TurretState(0.0, 0.0));
        table.put(220.0, new TurretState(0.0, 0.0));
        table.put(230.0, new TurretState(0.0, 0.0));
        table.put(240.0, new TurretState(0.0, 0.0));
        table.put(250.0, new TurretState(0.0, 0.0));
        table.put(260.0, new TurretState(0.0, 0.0));
        table.put(270.0, new TurretState(0.0, 0.0));
        table.put(280.0, new TurretState(0.0, 0.0));
        table.put(290.0, new TurretState(0.0, 0.0));
        table.put(300.0, new TurretState(0.0, 0.0));
        table.put(310.0, new TurretState(0.0, 0.0));
        table.put(320.0, new TurretState(0.0, 0.0));
        table.put(330.0, new TurretState(0.0, 0.0));
    }

    public TurretState get(double distance) {
        TurretState state = table.get(distance);
        if (state == null) {
            // Only happens if table is empty, but this keeps you safe.
            return new TurretState(0.0, 0.0);
        }
        return state;
    }

    public void clear() {
        table.clear();
    }
}
