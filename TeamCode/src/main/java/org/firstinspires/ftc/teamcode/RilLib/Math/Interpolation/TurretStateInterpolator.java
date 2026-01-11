package org.firstinspires.ftc.teamcode.RilLib.Math.Interpolation;

import org.firstinspires.ftc.teamcode.state.TurretState;

public class TurretStateInterpolator implements Interpolator<TurretState> {
    @Override
    public TurretState interpolate(TurretState a, TurretState b, double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        double rpm = a.rpm + (b.rpm - a.rpm) * t;
        double hood = a.hoodDeg + (b.hoodDeg - a.hoodDeg) * t;

        return new TurretState(rpm, hood);
    }
}
