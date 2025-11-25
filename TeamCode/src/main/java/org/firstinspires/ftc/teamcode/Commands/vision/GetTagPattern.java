package org.firstinspires.ftc.teamcode.Commands.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class GetTagPattern extends CommandBase {

    private final Vision vision;
    private boolean setPattern = false;

    public GetTagPattern (Vision vision) {
        this.vision = vision;

        addRequirements(vision);
    }

    @Override
    public void execute () {
    }

    @Override
    public boolean isFinished () {
        return setPattern;
    }

    @Override
    public void end (boolean interrupted) {
        if (RobotState.getInstance().getPattern() == null && !setPattern) {
            RobotState.getInstance().setPattern(new BallColor[] {
                    BallColor.PURPLE,
                    BallColor.PURPLE,
                    BallColor.GREEN
            }) ;
        }
    }

}
