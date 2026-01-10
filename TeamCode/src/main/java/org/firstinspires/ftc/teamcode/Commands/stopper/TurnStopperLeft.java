package org.firstinspires.ftc.teamcode.Commands.stopper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BallStoppers;

public class TurnStopperLeft extends CommandBase {
    private final BallStoppers ballStoppers;
    private final double leftAngle;

    public TurnStopperLeft(BallStoppers ballStoppers, double leftAngle) {
        this.ballStoppers = ballStoppers;
        this.leftAngle = leftAngle;

        addRequirements(ballStoppers);
    }

    @Override
    public void initialize() {
        ballStoppers.turnStopperLeft(leftAngle);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }
}
