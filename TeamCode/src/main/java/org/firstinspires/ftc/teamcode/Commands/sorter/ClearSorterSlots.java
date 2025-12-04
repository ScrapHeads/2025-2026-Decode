package org.firstinspires.ftc.teamcode.Commands.sorter;

import static org.firstinspires.ftc.teamcode.util.BallColor.EMPTY;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class ClearSorterSlots extends CommandBase {
    private final Sorter sorter;

    public ClearSorterSlots (Sorter sorter) {
        this.sorter = sorter;

        addRequirements(sorter);
    }

    @Override
    public void initialize() {
        BallColor[] ballColors = new BallColor[] {EMPTY, EMPTY, EMPTY};
        sorter.setSlots(ballColors);
    }

    @Override
    public void execute() { }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) { }
}
