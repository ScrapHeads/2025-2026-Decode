package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeLeft;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeRight;

/**
 * Command to run the Intake subsystem motor at a specific power.
 *
 * <p>Runs continuously until interrupted or canceled.
 */
public class RunBothIntakes extends CommandBase {

    private final IntakeLeft intakeLeft;
    private final IntakeRight intakeRight;
    private final double power;

    /**
     * Constructs a new RunIntakeCommand.
     *
     * @param intakeLeft the Intake subsystem instance
     * @param power the desired motor power (-1.0 to 1.0)
     */
    public RunBothIntakes(IntakeLeft intakeLeft, IntakeRight intakeRight, double power) {
        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
        this.power = power;
        addRequirements(intakeLeft);
        addRequirements(intakeRight);
    }

    @Override
    public void initialize() {
        intakeLeft.setLeftPower(power);
        intakeRight.setRightPower(power);
    }

    @Override
    public void end(boolean interrupted) {
//        intake.stopBoth();
    }

    @Override
    public boolean isFinished() {
        // Continuous command — runs until canceled
        return true;
    }
}
