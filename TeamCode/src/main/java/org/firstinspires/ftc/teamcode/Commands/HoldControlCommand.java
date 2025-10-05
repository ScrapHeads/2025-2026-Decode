package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HoldControl;

public class HoldControlCommand extends CommandBase {
    private final HoldControl holdControl;
    private final HoldControl.HoldPosition targetPosition;

    public HoldControlCommand(HoldControl holdControl, HoldControl.HoldPosition targetPosition) {
        this.holdControl = holdControl;
        this.targetPosition = targetPosition;
        addRequirements(holdControl);
    }

    @Override
    public void initialize() {
        holdControl.moveTo(targetPosition);
    }

    @Override
    public boolean isFinished() { return true; }
}
