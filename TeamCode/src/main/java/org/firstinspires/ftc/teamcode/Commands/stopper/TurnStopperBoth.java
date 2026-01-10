package org.firstinspires.ftc.teamcode.Commands.stopper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BallStoppers;

public class TurnStopperBoth extends CommandBase {
    private final BallStoppers ballStoppers;
    private final double leftAngle;
    private final double rightAngle;

    public TurnStopperBoth (BallStoppers ballStoppers, double leftAngle, double rightAngle) {
        this.ballStoppers = ballStoppers;
        this.leftAngle = leftAngle;
        this.rightAngle = rightAngle;

        addRequirements(ballStoppers);
    }

    @Override
    public void initialize() {
        ballStoppers.turnStopperBoth(leftAngle, rightAngle);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }
}
