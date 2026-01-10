package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeRight;

public class RunRightIntake extends CommandBase {
    private final IntakeRight intakeRight;
    private final double power;

    /**
     * Constructs a new RunIntakeCommand.
     *
     * @param intakeRight the Intake subsystem instance
     * @param power the desired motor power (-1.0 to 1.0)
     */
    public RunRightIntake(IntakeRight intakeRight, double power) {
        this.intakeRight = intakeRight;
        this.power = power;
        addRequirements(this.intakeRight);
    }

    @Override
    public void initialize() {
        intakeRight.setRightPower(power);
    }

    @Override
    public void end(boolean interrupted) {
//        intake.stopRight();
    }

    @Override
    public boolean isFinished() {
        // Continuous command — runs until canceled
        return true;
    }
}
