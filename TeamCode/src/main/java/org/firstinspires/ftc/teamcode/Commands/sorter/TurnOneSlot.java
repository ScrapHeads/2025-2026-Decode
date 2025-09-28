package org.firstinspires.ftc.teamcode.Commands.sorter;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class TurnOneSlot extends CommandBase {
    private final Sorter sorter;
    private double power;

    private long endNs = Sorter.STEP_MS;

    public TurnOneSlot(Sorter sorter, double power) {
        this.sorter = sorter;
        this.power = power;
        addRequirements(sorter);
    }

    @Override public void initialize() {
        sorter.setPower(power);
        sorter.advanceSlot(power);
        endNs = System.nanoTime() + (Sorter.STEP_MS * 1_000_000L);
    }

    @Override public boolean isFinished() {
        return System.nanoTime() >= endNs;
    }

    @Override public void end(boolean interrupted) {
        sorter.setPower(0);
    }
}
