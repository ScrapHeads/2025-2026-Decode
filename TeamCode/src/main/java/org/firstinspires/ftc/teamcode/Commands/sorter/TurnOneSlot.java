package org.firstinspires.ftc.teamcode.Commands.sorter;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

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
    private final int direction;

    // === Magnetic trigger state ===
    private boolean initialMagnetState;
    private boolean triggeredOnce;

    // Optional debounce timing (to prevent false double-triggers)
    private static final long DEBOUNCE_MS = 30;
    private long lastTriggerTime = 0;

    /**
     * @param sorter the sorter subsystem
     * @param direction either 1 turn CW or -1 turn CCW 0 does nothing
     * */
    public TurnOneSlot(Sorter sorter, int direction) {
        this.sorter = sorter;
        this.direction = direction;

        addRequirements(sorter);
    }

    @Override
    public void initialize() {
        if (direction == 0) {
            return;
        }

        sorter.setPower(direction);
        sorter.advanceSlot(direction);

        initialMagnetState = RobotState.getInstance().getMagSensorState();
        triggeredOnce = false;
        lastTriggerTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        boolean currentState = RobotState.getInstance().getMagSensorState();

        // Detect change in magnetic sensor state (rising/falling edge)
        if (currentState != initialMagnetState) {
            long now = System.currentTimeMillis();

            // Debounce: ensure a short delay before counting a trigger
            if (now - lastTriggerTime > DEBOUNCE_MS) {
                tele.addLine("in statement");
                triggeredOnce = true;
                lastTriggerTime = now;
                sorter.setPower(0);
            }

            // Update last known state
            initialMagnetState = currentState;
        }
    }

    @Override
    public boolean isFinished() {
        return triggeredOnce;
    }

    @Override
    public void end(boolean interrupted) {
        sorter.setPower(0);
    }
}
