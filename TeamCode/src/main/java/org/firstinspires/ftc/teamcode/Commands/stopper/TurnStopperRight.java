package org.firstinspires.ftc.teamcode.Commands.stopper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.BallStoppers;

public class TurnStopperRight extends CommandBase {
    private final BallStoppers ballStoppers;
    private final double rightAngle;

    public TurnStopperRight(BallStoppers ballStoppers, double rightAngle) {
        this.ballStoppers = ballStoppers;
        this.rightAngle = rightAngle;

        addRequirements(ballStoppers);
    }

    @Override
    public void initialize() {
        ballStoppers.turnStopperRight(rightAngle);
    }

    @Override
    public boolean isFinished() {
        return true; // instant command
    }
}
