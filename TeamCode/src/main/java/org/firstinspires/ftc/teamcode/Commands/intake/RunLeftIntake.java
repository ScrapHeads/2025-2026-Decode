package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class RunLeftIntake extends CommandBase {
    private final Intake intake;
    private final double power;

    /**
     * Constructs a new RunIntakeCommand.
     *
     * @param intake the Intake subsystem instance
     * @param power the desired motor power (-1.0 to 1.0)
     */
    public RunLeftIntake(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setLeftPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopLeft();
    }

    @Override
    public boolean isFinished() {
        // Continuous command — runs until canceled
        return false;
    }
}
