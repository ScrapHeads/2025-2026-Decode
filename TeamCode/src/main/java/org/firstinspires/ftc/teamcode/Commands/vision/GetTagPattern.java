package org.firstinspires.ftc.teamcode.Commands.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class GetTagPattern extends CommandBase {

    private final Vision vision;
    private boolean setPattern = false;

    public GetTagPattern (Vision vision) {
        this.vision = vision;
        vision.setPipeline(1);

        addRequirements(vision);
    }

    @Override
    public void execute () {
        setPattern = vision.detectPattern();
    }

    @Override
    public boolean isFinished () {
        return setPattern;
    }

    @Override
    public void end (boolean interrupted) {
        vision.setPipeline(0);
    }

}
