package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.BallStoppers;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeLeft;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

public class IntakeSorterNoEnd extends CommandBase {

    private final IntakeLeft intakeLeft;
    private final Sorter sorter;
    private final BallStoppers ballStoppers;

    private final double power;

    public IntakeSorterNoEnd(IntakeLeft intakeLeft, Sorter sorter, BallStoppers ballStoppers, double power) {
        this.intakeLeft = intakeLeft;
        this.sorter = sorter;
        this.ballStoppers = ballStoppers;

        this.power = power;
        addRequirements(intakeLeft);
        addRequirements(sorter);
    }

    @Override
    public void initialize() {
//        intakeLeft.setBothPower(power);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished () {
        return false;
//        return false;
    }

    @Override
    public void end(boolean interrupted) {
//        intakeLeft.setBothPower(0);
    }
}
