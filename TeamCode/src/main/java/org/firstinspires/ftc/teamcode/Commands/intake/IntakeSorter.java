package org.firstinspires.ftc.teamcode.Commands.intake;

import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class IntakeSorter extends CommandBase {

    private final Intake intake;
    private final Sorter sorter;
    private final HoldControl holdControl;

    private final double power;

    public IntakeSorter(Intake intake, Sorter sorter, HoldControl holdControl, double power) {
        this.intake = intake;
        this.sorter = sorter;
        this.holdControl = holdControl;

        this.power = power;
        addRequirements(intake);
        addRequirements(sorter);
    }

    @Override
    public void initialize() {
        holdControl.moveTo(HoldControl.HoldPosition.LOADING);
        intake.setPower(power);
    }

    @Override
    public void execute() {
        if (sorter.detectBallColor() != BallColor.EMPTY &&
                RobotState.getInstance().getBallColors()[sorter.getCurrentIndex()] == BallColor.EMPTY) {

            boolean initialMagnetState = RobotState.getInstance().getMagSensorState();
            long lastTriggerTime = System.currentTimeMillis();
            sorter.setPower(-.1);
            while (true) {
                boolean currentState = RobotState.getInstance().getMagSensorState();

                // Detect change in magnetic sensor state (rising/falling edge)
                if (currentState != initialMagnetState) {
                    long now = System.currentTimeMillis();

                    // Debounce: ensure a short delay before counting a trigger
                    if (now - lastTriggerTime > 30) {
                        tele.addLine("in statement");
                        sorter.setPower(0);
                        break;
                    }

                    // Update last known state
                    initialMagnetState = currentState;
                }
            }

            sorter.advanceSlot(1);
        }
    }

    @Override
    public boolean isFinished () {
        return RobotState.getInstance().hasEmpty();
//        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
        holdControl.moveTo(HoldControl.HoldPosition.TRANSPORT);
    }
}
