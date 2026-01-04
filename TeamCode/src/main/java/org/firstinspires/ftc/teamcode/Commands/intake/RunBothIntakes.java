package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Command to run the Intake subsystem motor at a specific power.
 *
 * <p>Runs continuously until interrupted or canceled.
 */
public class RunBothIntakes extends CommandBase {

    private final Intake intake;
    private final double power;

    /**
     * Constructs a new RunIntakeCommand.
     *
     * @param intake the Intake subsystem instance
     * @param power the desired motor power (-1.0 to 1.0)
     */
    public RunBothIntakes(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setBothPower(power);
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
