package org.firstinspires.ftc.teamcode.Commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherBall;

public class BallLauncher extends CommandBase {
    public final LauncherBall launcherBall;

    private final double power;



    public BallLauncher(LauncherBall launcherBall, double power) {
        this.launcherBall = launcherBall;
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
