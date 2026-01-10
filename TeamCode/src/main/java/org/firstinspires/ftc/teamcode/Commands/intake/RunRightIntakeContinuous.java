package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeRight;

public class RunRightIntakeContinuous extends CommandBase {
    private final IntakeRight intakeRight;
    private final double power;

    /**
     * Constructs a new RunIntakeCommand.
     *
     * @param intakeLeft the Intake subsystem instance
     * @param power the desired motor power (-1.0 to 1.0)
     */
    public RunRightIntakeContinuous(IntakeRight intakeLeft, double power) {
        this.intakeRight = intakeLeft;
        this.power = power;
        addRequirements(intakeLeft);
    }

    @Override
    public void initialize() {
        intakeRight.setRightPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        intakeRight.stopRight();
    }
}
