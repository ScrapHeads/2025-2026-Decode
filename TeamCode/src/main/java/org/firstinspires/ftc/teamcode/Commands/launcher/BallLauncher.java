package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class BallLauncher extends CommandBase {
    public final Launcher launcherBall;

    private final double power;



    public BallLauncher(Launcher launcher, double power) {
        this.launcherBall = launcher;
        this.power = power;
    }

    @Override
    public void initialize() {
        launcherBall.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
