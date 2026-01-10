package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeLeft;

public class RunLeftIntake extends CommandBase {
    private final IntakeLeft intakeLeft;
    private final double power;

    /**
     * Constructs a new RunIntakeCommand.
     *
     * @param intakeLeft the Intake subsystem instance
     * @param power the desired motor power (-1.0 to 1.0)
     */
    public RunLeftIntake(IntakeLeft intakeLeft, double power) {
        this.intakeLeft = intakeLeft;
        this.power = power;
        addRequirements(intakeLeft);
    }

    @Override
    public void initialize() {
        intakeLeft.setLeftPower(power);
    }

    @Override
    public void end(boolean interrupted) {
//        intake.stopLeft();
    }

    @Override
    public boolean isFinished() {
        // Continuous command — runs until canceled
        return true;
    }
}
