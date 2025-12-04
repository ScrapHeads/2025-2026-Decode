package org.firstinspires.ftc.teamcode.Commands.intake;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.tele;
import static org.firstinspires.ftc.teamcode.subsystems.HoldControl.HoldPosition.LOADING;
import static org.firstinspires.ftc.teamcode.subsystems.HoldControl.HoldPosition.TRANSPORT;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.HoldControl;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class IntakeSorterNoEnd extends CommandBase {

    private final Intake intake;
    private final Sorter sorter;
    private final HoldControl holdControl;

    private final double power;

    public IntakeSorterNoEnd(Intake intake, Sorter sorter, HoldControl holdControl, double power) {
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
        if (holdControl.getCurrentPosition() == TRANSPORT && sorter.isAtSetPoint()) {
            holdControl.moveTo(LOADING);
        }

        boolean isAtSetPoint = Math.abs(Math.abs(sorter.getCurrentPos()) - Math.abs(sorter.getTurnPos())) <= 800;

        if (sorter.getCurrentColor() != BallColor.EMPTY && isAtSetPoint) {
            holdControl.moveTo(TRANSPORT);
            sorter.turnOneSlotDirection(Sorter.CW_DIRECTION);
        }
    }

    @Override
    public boolean isFinished () {
        return false;
//        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
        holdControl.moveTo(TRANSPORT);
    }
}
