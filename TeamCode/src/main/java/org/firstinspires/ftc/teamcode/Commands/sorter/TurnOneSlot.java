package org.firstinspires.ftc.teamcode.Commands.sorter;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

import java.util.function.Supplier;

/**
 * Command to rotate the sorter exactly one slot using the magnetic sensor
 * for precise rotation detection rather than a fixed timer.
 *
 * <p>The command starts the rotation at the specified power and monitors
 * the magnetic sensor. When a full slot rotation is detected (sensor triggered
 * once), the sorter stops automatically.
 */
public class TurnOneSlot extends CommandBase {
    private final Sorter sorter;
    private int direction;
    private Supplier<Integer> directionSupplier;

    /**
     * @param sorter the sorter subsystem
     * @param direction either 1 turn CW or -1 turn CCW 0 does nothing
     * */
    public TurnOneSlot(Sorter sorter, int direction) {
        this.sorter = sorter;
        this.direction = direction;

        addRequirements(sorter);
    }

    public TurnOneSlot(Sorter sorter, Supplier<Integer> directionSupplier) {
        this.sorter = sorter;
        this.directionSupplier = directionSupplier;

        addRequirements(sorter);
    }

    @Override
    public void initialize() {
        int dir = (directionSupplier != null) ? directionSupplier.get() : direction;
        sorter.turnOneSlotDirection(dir);
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
